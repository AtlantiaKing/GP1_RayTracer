//External includes
#include "SDL.h"
#include "SDL_surface.h"
#include <thread>
#include <future> // Async Stuff
#include <ppl.h> // Parallel Stuff

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"

using namespace dae;

//#define ASYNC
#define PARALLEL_FOR
//#define USE_SOFT_SHADOWS

Renderer::Renderer(SDL_Window* pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
	m_AspectRatio = float(m_Width) / m_Height;
}

void Renderer::Render(Scene* pScene) const
{
	// Get information 
	Camera& camera{ pScene->GetCamera() };
	auto& materials{ pScene->GetMaterials() };
	auto& lights{ pScene->GetLights() };

	// Calculate the current matrix for the camera transform
	camera.CalculateCameraToWorld();

	// The number of pixels that are going to be shown
	const unsigned int nrPixels{ static_cast<unsigned int>(m_Width * m_Height) };

#if defined(ASYNC)
	// Async Logic
	const unsigned int nrCores{ std::thread::hardware_concurrency() };
	std::vector<std::future<void>> asyncFutures{};

	const unsigned int nrPixelsPerTask{ nrPixels / nrCores };
	unsigned int nrUnassignedPixels{ nrPixels % nrCores };
	unsigned int curPixelIdx{};

	for (unsigned int coreIdx{}; coreIdx < nrCores; ++coreIdx)
	{
		unsigned int taskSize{ nrPixelsPerTask };
		if (nrUnassignedPixels > 0)
		{
			++taskSize;
			--nrUnassignedPixels;
		}

		asyncFutures.push_back(
			std::async(std::launch::async, [=, this]
				{
					const unsigned int endPixelIdx{ curPixelIdx + taskSize };
					for (unsigned int pixelIdx{ curPixelIdx }; pixelIdx < endPixelIdx; ++pixelIdx)
					{
						RenderPixel(pScene, pixelIdx, camera, lights, materials);
					}
				})
		);

		curPixelIdx += taskSize;
	}

	for (const std::future<void>& f : asyncFutures)
	{
		f.wait();
	}

#elif defined(PARALLEL_FOR)
	// Parallel For Logic
	concurrency::parallel_for(0u, nrPixels,
		[=, this](int i)
		{
			RenderPixel(pScene, i, camera, lights, materials);
		});
#else
	// Synchronous Logic
	for (unsigned int i{}; i < nrPixels; ++i)
	{
		RenderPixel(pScene, i, camera, lights, materials);
	}
#endif

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

void dae::Renderer::RenderPixel(Scene* pScene, unsigned int pixelIndex, const Camera& camera, const std::vector<Light>& lights, const std::vector<Material*>& materials) const
{
	// Calculate the row and column from pixelIndex
	const int px{ static_cast<int>(pixelIndex) % m_Width };
	const int py{ static_cast<int>(pixelIndex) / m_Width };

	// Calculate the raster cordinates in camera space
	const float cx{ ((2.0f * (px + 0.5f) / m_Width - 1.0f) * m_AspectRatio) * camera.fovMultiplier };
	const float cy{ (1.0f - 2.0f * (py + 0.5f) / m_Height) * camera.fovMultiplier };

	// Calculate the direction from the camera to the raster
	const Vector3 rasterDirection{ cx, cy, 1.0f };

	// Calculate and normalize the ray direction
	Vector3 rayDirection{ camera.cameraToWorld.TransformVector(rasterDirection) };
	rayDirection.Normalize();

	// Create a ray from the camera to the raster
	Ray viewRay{ camera.origin, rayDirection };

	// Initialize the pixel color (default = black)
	ColorRGB finalColor{ };

	// Find the closest hit in the scene to the camera and save the hit information
	HitRecord closestHit{ };
	pScene->GetClosestHit(viewRay, closestHit);

	// If the ray does hits anything, calculate shadows and lighting
	if (closestHit.didHit)
	{
		// Offset the origin point to avoid hitting the same object again
		const Vector3 offsetPosition{ closestHit.origin + closestHit.normal * 0.001f };

		// For every light
		for (const Light& light : lights)
		{
			// Calculate the direction between the point to the light position
			Vector3 lightDirection{ LightUtils::GetDirectionToLight(light, offsetPosition) };

			// Calculate the distance between the point and the light and normalize the light direction
			const float lightDistance{ lightDirection.Normalize() };


#ifdef USE_SOFT_SHADOWS
			// The amount of light that reaches a pixel
			float lightAmount{ 1.0f };

			// If shadows are not enabled, do nothing
			if (m_AreShadowsEnabled)
			{
				// Find a perpendicular vector to the light direction
				// https://math.stackexchange.com/questions/137362/how-to-find-perpendicular-vector-to-another-vector
				Vector3 lightRadiusDirection{ lightDirection.z, lightDirection.z, -lightDirection.x - lightDirection.y };
				lightRadiusDirection.Normalize();

				// Define the size of the grid that will be checked
				const int lightGridSize{ 5 };

				// Find the X and Y directions of the grid (scaled to be able to scale them using an x/y index)
				const Vector3 perpendicularLightDirectionX{ lightRadiusDirection * (light.radius / Square(lightDistance)) / (lightGridSize / 2) };
				const Vector3 perpendicularLightDirectionY{ Vector3::Cross(lightDirection, perpendicularLightDirectionX) };

				// Calculate the largest index in the grid
				const int maxGridCellPosition{ lightGridSize / 2 };

				// Create a ray from the point to the light grid
				// Set the max of the ray to the distance between the light and the point; 
				//		otherwise the ray will always hit something (e.g. a infinite plane)
				Ray lightRay{ offsetPosition, lightDirection, 0.0001f, lightDistance };

				// For each grid cell
				for (int gridX{ -maxGridCellPosition }; gridX <= maxGridCellPosition; ++gridX)
				{
					for (int gridY{ -maxGridCellPosition }; gridY <= maxGridCellPosition; ++gridY)
					{
						// Calculate the direction from the camera to the grid
						Vector3 subLightDirection{ lightDirection + perpendicularLightDirectionX * static_cast<float>(gridX) + perpendicularLightDirectionY * static_cast<float>(gridY) };
						subLightDirection.Normalize();

						lightRay.direction = subLightDirection;

						// If there is shadow: Darken the pixel and continue on to the next pixel
						if (pScene->DoesHit(lightRay))
						{
							lightAmount -= 1.0f / (lightGridSize * lightGridSize);
						}
					}
				}

				// If the amount of light is less then a certain threshold, there is a full shadow and it should continue to the next light
				const float hardShadowThreshold{ 0.05f };
				if (lightAmount < hardShadowThreshold)
				{
					continue;
				}
			}
#else
			// If shadows are not enabled, do nothing
			if (m_AreShadowsEnabled)
			{
				// Create a ray from the point to the light position
						// Set the max of the ray to the distance between the light and the point; 
						//		otherwise the ray will always hit something (e.g. a infinite plane)
				Ray lightRay{ offsetPosition, lightDirection, 0.0001f, lightDistance };

				// If there is shadow: continue to the next light
				if (pScene->DoesHit(lightRay))
				{
					continue;
				}
			}
#endif

			switch (m_CurrentLightingMode)
			{
			case LightingMode::ObservedArea:	// Only show Lambert Cosine Law 
			{
				const float lightNormalAngle{ Vector3::DotClamped(closestHit.normal, lightDirection) };
#ifdef USE_SOFT_SHADOWS
				finalColor += ColorRGB{ lightNormalAngle, lightNormalAngle, lightNormalAngle } *lightAmount;
#else
				finalColor += ColorRGB{ lightNormalAngle, lightNormalAngle, lightNormalAngle };
#endif
			}
			break;
			case LightingMode::Radiance:
			{
				// Only show Radiance
#ifdef USE_SOFT_SHADOWS
				finalColor += LightUtils::GetRadiance(light, closestHit.origin) * lightAmount;
#else
				finalColor += LightUtils::GetRadiance(light, closestHit.origin);
#endif
			}
			break;
			case LightingMode::BRDF:			// Only show BRDF of a material
			{
				const ColorRGB brdf{ materials[closestHit.materialIndex]->Shade(closestHit, lightDirection, -rayDirection) };
#ifdef USE_SOFT_SHADOWS
				finalColor += brdf * lightAmount;
#else
				finalColor += brdf;
#endif
			}
			break;
			case LightingMode::Combined:		// Show everything combined (default)
			{
				const float lightNormalAngle{ Vector3::DotClamped(closestHit.normal, lightDirection) };
				const ColorRGB radiance{ LightUtils::GetRadiance(light, closestHit.origin) };
				const ColorRGB brdf{ materials[closestHit.materialIndex]->Shade(closestHit, lightDirection, -rayDirection) };
#ifdef USE_SOFT_SHADOWS
				finalColor += radiance * brdf * lightNormalAngle * lightAmount;
#else
				finalColor += radiance * brdf * lightNormalAngle;
#endif
			}
			break;
			}
		}
	}

	//Update Color in Buffer
	finalColor.MaxToOne();

	m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
		static_cast<uint8_t>(finalColor.r * 255),
		static_cast<uint8_t>(finalColor.g * 255),
		static_cast<uint8_t>(finalColor.b * 255));
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void dae::Renderer::CycleLightingMode()
{
	const int maxLightingValue = static_cast<int>(LightingMode::Combined) + 1;
	int curLightingValue = static_cast<int>(m_CurrentLightingMode);

	m_CurrentLightingMode = static_cast<LightingMode>(++curLightingValue % maxLightingValue);

	switch (m_CurrentLightingMode)
	{
	case dae::Renderer::LightingMode::ObservedArea:
		std::cout << "[LIGHTMODE CHANGE] Observed Area\n";
		break;
	case dae::Renderer::LightingMode::Radiance:
		std::cout << "[LIGHTMODE CHANGE] Radiance\n";
		break;
	case dae::Renderer::LightingMode::BRDF:
		std::cout << "[LIGHTMODE CHANGE] BRDF\n";
		break;
	case dae::Renderer::LightingMode::Combined:
		std::cout << "[LIGHTMODE CHANGE] Combined\n";
		break;
	}
}

void dae::Renderer::ToggleShadows()
{
	m_AreShadowsEnabled = !m_AreShadowsEnabled;

	if (m_AreShadowsEnabled)
	{
		std::cout << "[SHADOWS CHANGE] ON\n";
	}
	else
	{
		std::cout << "[SHADOWS CHANGE] OFF\n";
	}
}

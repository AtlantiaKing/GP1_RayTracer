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
	const int px = pixelIndex % m_Width;
	const int py = pixelIndex / m_Width;

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

			if (m_AreShadowsEnabled)
			{
				// Create a ray from the point to the light position
				// Set the max of the ray to the distance between the light and the point; 
				//		otherwise the ray will always hit something (e.g. a infinite plane)
				Ray lightRay{ offsetPosition, lightDirection, 0.0001f, lightDistance };

				// If there is shadow: Darken the pixel and continue on to the next pixel
				if (pScene->DoesHit(lightRay))
				{
					continue;
				}
			}

			const float lightNormalAngle{ std::max(Vector3::Dot(closestHit.normal, lightDirection), 0.0f) };

			switch (m_CurrentLightingMode)
			{
			case LightingMode::ObservedArea:	// Only show Lambert Cosine Law 
				finalColor += ColorRGB{ lightNormalAngle, lightNormalAngle, lightNormalAngle };
				break;
			case LightingMode::Radiance:		// Only show Radiance
				finalColor += LightUtils::GetRadiance(light, closestHit.origin);
				break;
			case LightingMode::BRDF:			// Only show BRDF of a material
			{
				const ColorRGB brdf{ materials[closestHit.materialIndex]->Shade(closestHit, lightDirection, -rayDirection) };
				finalColor += brdf;
			}
			break;
			case LightingMode::Combined:		// Show everything combined (default)
			{
				const ColorRGB radiance{ LightUtils::GetRadiance(light, closestHit.origin) };
				const ColorRGB brdf{ materials[closestHit.materialIndex]->Shade(closestHit, lightDirection, -rayDirection) };
				finalColor += radiance * brdf * lightNormalAngle;
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

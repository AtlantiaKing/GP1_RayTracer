//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"

using namespace dae;

Renderer::Renderer(SDL_Window * pWindow) :
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
	Camera& camera = pScene->GetCamera();
	auto& materials = pScene->GetMaterials();

	// Retrieve the current matrix for the camera transform
	const Matrix cameraInWorldSpace{ camera.CalculateCameraToWorld() };

	// For each pixel
	for (int px{}; px < m_Width; ++px)
	{
		for (int py{}; py < m_Height; ++py)
		{
			// Calculate the raster cordinates in camera space
			const float cx{ ((2.0f * (px + 0.5f) / m_Width - 1.0f) * m_AspectRatio) * camera.fovMultiplier };
			const float cy{ (1.0f - 2.0f * (py + 0.5f) / m_Height) * camera.fovMultiplier };

			// Calculate the direction from the camera to the raster
			const Vector3 rasterDirection{ cx, cy, 1.0f };

			// Calculate and normalize the ray direction
			Vector3 rayDirection{ cameraInWorldSpace.TransformVector(rasterDirection) };
			rayDirection.Normalize();

			// Create a ray from the camera to the raster
			const Ray viewRay{ camera.origin, rayDirection };

			// Initialize the pixel color (default = black)
			ColorRGB finalColor{ };

			// Find the closest hit in the scene to the camera and save the hit information
			HitRecord closestHit{ };
			pScene->GetClosestHit(viewRay, closestHit);

			// The total percentage of light on a pixel
			float lightValue{ 1.0f }; 

			// If the ray hits anything, set finalColor to the hit material color
			if (closestHit.didHit)
			{
				// Set the color to the shading of the current object
				//finalColor = materials[closestHit.materialIndex]->Shade();

				// For every light
				for (const Light& light : pScene->GetLights())
				{
					// Offset the origin point to avoid hitting the same object again
					const Vector3 offsetPosition{ closestHit.origin + closestHit.normal * 0.001f };

					// Calculate the direction between the point to the light position
					Vector3 lightDirection{ LightUtils::GetDirectionToLight(light, offsetPosition) };

					// Calculate the distance between the point and the light and normalize the light direction
					const float lightDistance{ lightDirection.Normalize() };

					if (m_AreShadowsEnabled)
					{
						// Create a ray from the point to the light position
						Ray lightRay{ offsetPosition, lightDirection };
						// Set the max of the ray to the distance between the light and the point; 
						//		otherwise the ray will always hit something (e.g. a infinite plane)
						lightRay.max = lightDistance;

						// If there is shadow: Darken the pixel and continue on to the next pixel
						if (pScene->DoesHit(lightRay))
						{
							lightValue *= 0.95f;
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

			// The apply the light percentage to the current color
			finalColor *= lightValue;

			m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
				static_cast<uint8_t>(finalColor.r * 255),
				static_cast<uint8_t>(finalColor.g * 255),
				static_cast<uint8_t>(finalColor.b * 255));
		}
	}

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
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
}

bool dae::Renderer::DoesPointHaveShadow(Scene* pScene, const Vector3& point) const
{
	// For every light
	for (const Light& light : pScene->GetLights())
	{
		// Calculate the direction between the point to the light position
		Vector3 lightDirection{ LightUtils::GetDirectionToLight(light, point) };

		// Calculate the distance between the point and the light and normalize the light direction
		const float lightDistance{ lightDirection.Normalize() };

		// Create a ray from the point to the light position
		Ray lightRay{ point, lightDirection };
		// Set the max of the ray to the distance between the light and the point; 
		//		otherwise the ray will always hit something (e.g. a infinite plane)
		lightRay.max = lightDistance;

		// Return wether the ray hits an object or not
		if (pScene->DoesHit(lightRay))
		{
			return true;
		}
	}
	return false;
}

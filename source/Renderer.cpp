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
	Camera& camera = pScene->GetCamera();
	auto& materials = pScene->GetMaterials();

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
			//rayDirection.Normalize();

			// Create a ray from the camera to the raster
			const Ray viewRay{ camera.origin, rayDirection };

			// Initialize the pixel color (default = black)
			ColorRGB finalColor{ };

			// Find the closest hit in the scene to the camera and save the hit information
			HitRecord closestHit{ };
			pScene->GetClosestHit(viewRay, closestHit);

			// If the ray hits anything, set finalColor to the hit material color
			if (closestHit.didHit)
			{
				finalColor = materials[closestHit.materialIndex]->Shade();

				// Offset the origin a really small amount to avoid hitting the object itself
				const Vector3 shadePosition{ closestHit.origin + closestHit.normal * 0.001f };

				// If an object is obstructing the line between the light and the hit point; draw a hard shadow
				if (DoesPointHaveShadow(pScene, shadePosition))
				{
					finalColor *= 0.5f;
				}
			}

			//Update Color in Buffer
			finalColor.MaxToOne();

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

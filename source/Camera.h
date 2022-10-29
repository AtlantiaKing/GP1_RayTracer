#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"
#include <iostream>

namespace dae
{
	struct Camera
	{
		Camera() = default;

		Camera(const Vector3& _origin, float _fovAngle)
			: origin{_origin}
		{
			SetFovAngle(_fovAngle);
		}


		Vector3 origin{};
		float fovAngle{ 90.f };
		float fovMultiplier{ 1.0f };

		Vector3 forward{Vector3::UnitZ};
		Vector3 up{Vector3::UnitY};
		Vector3 right{Vector3::UnitX};

		float totalPitch{0.f};
		float totalYaw{0.f};

		Matrix cameraToWorld{};

		Matrix CalculateCameraToWorld()
		{
			right = Vector3::Cross(Vector3::UnitY, forward);
			up = Vector3::Cross(forward, right);

			cameraToWorld = Matrix
			{
				right,
				up,
				forward,
				origin
			};

			return cameraToWorld;
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();

			// Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);

			// Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);

			// Speed and limit constants
			const float keyboardMovementSpeed{ 10.0f };
			const float fovChangeSpeed{ 50.0f };
			const float minFov{ 30.0f };
			const float maxFov{ 170.0f };
			const float mouseMovementSpeed{ 2.0f };
			const float angularSpeed{ 10.0f * TO_RADIANS };

			// The total movement of this frame
			Vector3 direction{};

			// Calculate new position with keyboard inputs
			direction += (pKeyboardState[SDL_SCANCODE_W] || pKeyboardState[SDL_SCANCODE_Z]) * forward * keyboardMovementSpeed * deltaTime;
			direction -= pKeyboardState[SDL_SCANCODE_S] * forward * keyboardMovementSpeed * deltaTime;
			direction -= (pKeyboardState[SDL_SCANCODE_Q] || pKeyboardState[SDL_SCANCODE_A]) * right * keyboardMovementSpeed * deltaTime;
			direction += pKeyboardState[SDL_SCANCODE_D] * right * keyboardMovementSpeed * deltaTime;

			// Calculate new fov with keyboard inputs
			float newFovAngle{ fovAngle };
			newFovAngle += pKeyboardState[SDL_SCANCODE_LEFT] * fovChangeSpeed * deltaTime;
			newFovAngle -= pKeyboardState[SDL_SCANCODE_RIGHT] * fovChangeSpeed * deltaTime;

			const float fovDifference{ newFovAngle - fovAngle };

			// Clamp the new fov angle and calculate the tangent
			if (fovDifference > FLT_EPSILON || fovDifference < -FLT_EPSILON)
			{
				if (newFovAngle < minFov)
				{
					newFovAngle = minFov;
				}
				else if (newFovAngle > maxFov)
				{
					newFovAngle = maxFov;
				}
				SetFovAngle(newFovAngle);
			}

			// Calculate new position and rotation with mouse inputs
			switch (mouseState)
			{
			case SDL_BUTTON_LMASK: // LEFT CLICK
				direction -= forward * (mouseY * mouseMovementSpeed * deltaTime);
				totalYaw += mouseX * angularSpeed * deltaTime;
				break;
			case SDL_BUTTON_RMASK: // RIGHT CLICK
				totalYaw += mouseX * angularSpeed * deltaTime;
				totalPitch -= mouseY * angularSpeed * deltaTime;
				break;
			case SDL_BUTTON_X2: // BOTH CLICK
				direction.y -= mouseY * mouseMovementSpeed * deltaTime;
				break;
			}

			// Speed up all movement when the shift button is pressed
			const float speedUpFactor{ 4.0f };
			direction *= 1.0f + pKeyboardState[SDL_SCANCODE_LSHIFT] * (speedUpFactor - 1.0f);

			// Apply the direction to the current position
			origin += direction;

			// Calculate the rotation matrix with the new pitch and yaw
			Matrix rotationMatrix = Matrix::CreateRotationX(totalPitch) * Matrix::CreateRotationY(totalYaw);

			// Calculate the new forward vector with the new pitch and yaw
			forward = rotationMatrix.TransformVector(Vector3::UnitZ);
			forward.Normalize();
		}

		void SetFovAngle(float newFovAngle)
		{
			fovAngle = newFovAngle;
			fovMultiplier = tanf(TO_RADIANS * newFovAngle / 2.0f);
		}
	};
}

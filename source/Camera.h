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

			Matrix worldMatrix
			{
				right,
				up,
				forward,
				origin
			};
			return worldMatrix;
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();

			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);


			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);

			const float keyboardMovementSpeed{ 10.0f };
			const float mouseMovementSpeed{ 2.0f };
			const float angularSpeed{ 10.0f * TO_RADIANS };

			if (pKeyboardState[SDL_SCANCODE_W] || pKeyboardState[SDL_SCANCODE_Z])
			{
				origin += forward * keyboardMovementSpeed * deltaTime;
			}
			else if (pKeyboardState[SDL_SCANCODE_S])
			{
				origin -= forward * keyboardMovementSpeed * deltaTime;
			}
			
			if (pKeyboardState[SDL_SCANCODE_Q] || pKeyboardState[SDL_SCANCODE_A])
			{
				origin -= right * keyboardMovementSpeed * deltaTime;
			}
			else if (pKeyboardState[SDL_SCANCODE_D])
			{
				origin += right * keyboardMovementSpeed * deltaTime;
			}
			
			switch (mouseState)
			{
			case SDL_BUTTON_LEFT: // LEFT CLICK
				origin.z -= mouseY * mouseMovementSpeed * deltaTime;
				totalYaw += mouseX * angularSpeed * deltaTime;
				break;
			case SDL_BUTTON_X1: // RIGHT CLICK
				totalYaw += mouseX * angularSpeed * deltaTime;
				totalPitch -= mouseY * angularSpeed * deltaTime;
				break;
			case SDL_BUTTON_X2: // BOTH CLICK
				origin.y -= mouseY * mouseMovementSpeed * deltaTime;
				break;
			}

			Matrix rotationMatrix = Matrix::CreateRotation(totalPitch, totalYaw, 0.0f);

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

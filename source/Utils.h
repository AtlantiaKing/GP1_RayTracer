#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"

namespace dae
{
	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
#pragma region Sphere HitTest Analytic
			// Analytic

			// Calculate a vector from the origin of the 
			const Vector3 raySphereVector{ ray.origin - sphere.origin };

			// Calculate all parts of the quadratic formula
			const float a{ Vector3::Dot(ray.direction, ray.direction) };
			const float b{ Vector3::Dot(2.0f * ray.direction, raySphereVector) };
			const float c{ Vector3::Dot(raySphereVector, raySphereVector) - Square(sphere.radius) };

			// Calculate the discriminant of the quadratic formulae
			const float discriminant{ Square(b) - 4.0f * a * c };

			// If discriminant is negative, nothing has been hit
			if (discriminant < 0)
			{
				return false;
			}

			if (!ignoreHitRecord)
			{
				const float sqrtDiscriminant{ sqrtf(discriminant) };

				// Calculate the distance from the ray origin to the ray hit point
				float t{ };

				// If discriminant == 0, only one hit point exists
				// Else, calculate both hit point distances and use the smallest
				if (discriminant < FLT_EPSILON)
				{
					t = -b / (2.0f * a);
				}
				else
				{
					t = (-b - sqrtDiscriminant) / (2.0f * a);
					if (t < ray.min || t > ray.max)
					{
						t = (-b + sqrtDiscriminant) / (2.0f * a);
						if (t < ray.min || t > ray.max)
						{
							// If both t values are less then zero, nothing is visible
							return false;
						}
					}
				}

				// Calculate the normal of the hit point
				const Vector3 hitPoint{ ray.origin + ray.direction * t };
				const Vector3 hitNormal{ sphere.origin, hitPoint };

				// Set the hit record to the calculated information
				hitRecord.normal = hitNormal.Normalized();
				hitRecord.origin = ray.origin;
				hitRecord.didHit = true;
				hitRecord.materialIndex = sphere.materialIndex;
				hitRecord.t = t;
			}

			return true;
#pragma endregion

#pragma region Sphere HitTest Geometric
			//// Geometric
			//
			//// Calculate the vector from the ray to the sphere
			//const Vector3 raySphereVector{ sphere.origin - ray.origin };
			//// Project the vector from ray to the sphere on the ray
			//const Vector3 projectedRaySphereVector{ Vector3::Project(raySphereVector, ray.direction) };
			//// Calculate the distance between the origin of the sphere and the projected origin of the sphere on the ray
			//const Vector3 sphereRayDistanceVector{ raySphereVector - projectedRaySphereVector };
			//const float sphereRayDistanceSqr{ sphereRayDistanceVector.SqrMagnitude() };
			//// Calculate the distance between the projected origin of the sphere and the hit point on the circle
			//const float rayHitDistanceInCirlceSqr{ Square(sphere.radius) - sphereRayDistanceSqr };

			//// If squared distance is less then zero, nothing is hit
			//if (rayHitDistanceInCirlceSqr < 0)
			//{
			//	return false;
			//}

			//if (!ignoreHitRecord)
			//{
			//	// Calculate the distance from the ray origin to the ray hit point

			//	// Calculate the distance from the projected origin of the circle to the ray origin
			//	const float projectedRaySphereDistance{ projectedRaySphereVector.Magnitude() };

			//	float rayDistance{ };
			//	// If the distance between the hit point and the projected origin is zero, the projected origin IS the hit point
			//	// Else, calculate both hit point distances and use the smallest
			//	if (rayHitDistanceInCirlceSqr < FLT_EPSILON)
			//	{
			//		rayDistance = projectedRaySphereDistance;
			//	}
			//	else
			//	{
			//		const float rayHitDistanceInCirlce{ sqrt(rayHitDistanceInCirlceSqr) };
			//		const float t1{ projectedRaySphereDistance - rayHitDistanceInCirlce };
			//		const float t2{ projectedRaySphereDistance + rayHitDistanceInCirlce };
			//		rayDistance = (t1 < t2&& t1 >= 0) ? t1 : t2;
			//	}
			//
			//	// If the raydistance is less then zero, nothing is visible
			//	if (rayDistance < 0)
			//	{
			//		return false;
			//	}
			//	
			//	// Calculate the normal of the hit point
			//	const Vector3 hitPoint{ ray.origin + ray.direction * rayDistance };
			//	const Vector3 hitNormal{ sphere.origin, hitPoint };
			//
			//	// Set the hit record to the calculated information
			//	hitRecord.normal = hitNormal.Normalized();
			//	hitRecord.origin = ray.origin;
			//	hitRecord.didHit = true;
			//	hitRecord.materialIndex = sphere.materialIndex;
			//	hitRecord.t = rayDistance;
			//}

			//return true;
#pragma endregion
		}

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Sphere(sphere, ray, temp, true);
		}
#pragma endregion
#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			// Calculate a vector on the plane
			const Vector3 planeRayDistance{ plane.origin - ray.origin };
			// Calculate the distance from ray hit
			const float t = Vector3::Dot(planeRayDistance, plane.normal) / Vector3::Dot(ray.direction, plane.normal);

			// If the distance is less then zero; the plane is not visible
			if (t < ray.min || t > ray.max)
			{
				return false;
			}

			// Set the hit record to the calculated information
			if (!ignoreHitRecord)
			{
				// Set the hit record to the calculated information
				hitRecord.normal = plane.normal;
				hitRecord.origin = ray.origin + ray.direction * t;
				hitRecord.t = t;
				hitRecord.didHit = true;
				hitRecord.materialIndex = plane.materialIndex;
			}

			return true;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion
#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//todo W5
			assert(false && "No Implemented Yet!");
			return false;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//todo W5
			assert(false && "No Implemented Yet!");
			return false;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_TriangleMesh(mesh, ray, temp, true);
		}
#pragma endregion
	}

	namespace LightUtils
	{
		//Direction from target to light
		inline Vector3 GetDirectionToLight(const Light& light, const Vector3 origin)
		{
			//todo W3
			assert(false && "No Implemented Yet!");
			return {};
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			//todo W3
			assert(false && "No Implemented Yet!");
			return {};
		}
	}

	namespace Utils
	{
		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vector3>& positions, std::vector<Vector3>& normals, std::vector<int>& indices)
		{
			std::ifstream file(filename);
			if (!file)
				return false;

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;
					positions.push_back({ x, y, z });
				}
				else if (sCommand == "f")
				{
					float i0, i1, i2;
					file >> i0 >> i1 >> i2;

					indices.push_back((int)i0 - 1);
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');

				if (file.eof()) 
					break;
			}

			//Precompute normals
			for (uint64_t index = 0; index < indices.size(); index += 3)
			{
				uint32_t i0 = indices[index];
				uint32_t i1 = indices[index + 1];
				uint32_t i2 = indices[index + 2];

				Vector3 edgeV0V1 = positions[i1] - positions[i0];
				Vector3 edgeV0V2 = positions[i2] - positions[i0];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2);

				if(isnan(normal.x))
				{
					int k = 0;
				}

				normal.Normalize();
				if (isnan(normal.x))
				{
					int k = 0;
				}

				normals.push_back(normal);
			}

			return true;
		}
#pragma warning(pop)
	}
}
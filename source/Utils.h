#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"

#define MAYA_IMPORT

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

			const float sqrtDiscriminant{ sqrtf(discriminant) };

			// Calculate the distance from the ray origin to the ray hit point
			float t{ };

			const float discriminantDivisor{ 1.0f / (2.0f * a) };

			// Calculate both hit point distances and use the smallest
			t = (-b - sqrtDiscriminant)  * discriminantDivisor;
			if (t < ray.min || t > ray.max)
			{
				t = (-b + sqrtDiscriminant) * discriminantDivisor;
				if (t < ray.min || t > ray.max)
				{
					// If both t values are less then ray.min or more then ray.max, nothing is visible
					return false;
				}
			}

			// If no hitrecord needs to be calculated, return true
			if (ignoreHitRecord)
			{
				return true;
			}

			// Calculate the normal of the hit point
			const Vector3 hitPoint{ ray.origin + ray.direction * t };
			const Vector3 hitNormal{ sphere.origin, hitPoint };

			// Set the hit record to the calculated information
			hitRecord.normal = hitNormal.Normalized();
			hitRecord.origin = hitPoint;
			hitRecord.didHit = true;
			hitRecord.materialIndex = sphere.materialIndex;
			hitRecord.t = t;

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

			// If no hitrecord needs to be calculated, return true
			if (ignoreHitRecord)
			{
				return true;
			}

			// Set the hit record to the calculated information
			hitRecord.normal = plane.normal;
			hitRecord.origin = ray.origin + ray.direction * t;
			hitRecord.t = t;
			hitRecord.didHit = true;
			hitRecord.materialIndex = plane.materialIndex;

			return true;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion
#pragma region Triangle HitTest
		inline bool IsToRightSideOfEdge(const Vector3& point, const Vector3& v0, const Vector3& v1, const Vector3& normal)
		{
			const Vector3 edge{ v1 - v0 };
			const Vector3 startToPoint{ point - v0 };

			const Vector3 edgePointCross{ Vector3::Cross(edge, startToPoint) };

			return Vector3::Dot(edgePointCross, normal) > 0;
		}

		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			// Calculate the dot product with the triangle normal and the view direction
			const float viewNormalDot{ Vector3::Dot(ray.direction, triangle.normal) };

			// If the camera looks perpendicular on the normal, the triangle is not visible
			if (viewNormalDot > -FLT_EPSILON && viewNormalDot < FLT_EPSILON) return false;

			// Get the correct current cullmode (the cullmode needs to be flipped for shadow rays)
			TriangleCullMode curCullMode{ triangle.cullMode };
			if (ignoreHitRecord)
			{
				switch (curCullMode)
				{
				case dae::TriangleCullMode::FrontFaceCulling:
					curCullMode = TriangleCullMode::BackFaceCulling;
					break;
				case dae::TriangleCullMode::BackFaceCulling:
					curCullMode = TriangleCullMode::FrontFaceCulling;
					break;
				}
			}

			// When the camera looks to the culled side of a triangle, return false
			switch (curCullMode)
			{
			case dae::TriangleCullMode::FrontFaceCulling:
				if (viewNormalDot < 0) return false;
				break;
			case dae::TriangleCullMode::BackFaceCulling:
				if (viewNormalDot > 0) return false;
				break;
			}

			// Source: https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

			// Calculate the edges of the triangle
			const Vector3 edge1{ triangle.v1 - triangle.v0 };
			const Vector3 edge2{ triangle.v2 - triangle.v0 };

			const Vector3 h{ Vector3::Cross(ray.direction, edge2) };

			const float f{ 1.0f / Vector3::Dot(edge1, h) };
			const Vector3 s{ ray.origin - triangle.v0 };
			const float u{ f * Vector3::Dot(s,h) };

			if (u < 0.0f || u > 1.0f) return false;

			const Vector3 q{ Vector3::Cross(s, edge1) };
			const float v{ f * Vector3::Dot(ray.direction, q) };

			if (v < 0.0f || u + v > 1.0f) return false;

			// At this stage we can compute t to find out where the intersection point is on the line.
			const float t{ f * Vector3::Dot(edge2, q) };

			// If the distance is less then zero; the triangle is not visible
			if (t < ray.min || t > ray.max) return false;

			// If hit records needs to be ignored, just return true
			if (ignoreHitRecord) return true;

			hitRecord.t = t;
			hitRecord.origin = ray.origin + ray.direction * t;
			hitRecord.normal = triangle.normal;
			hitRecord.materialIndex = triangle.materialIndex;
			hitRecord.didHit = true;

			return true;

			//// Calculate the center of the triangle
			//Vector3 center{ (triangle.v0 + triangle.v1 + triangle.v2) / 3.0f };
			//
			//// Calculate a vector from the camera to the triangle
			//const Vector3 planeRayDistance{ center - ray.origin };

			//// Calculate the distance from ray hit
			//const float t = Vector3::Dot(planeRayDistance, triangle.normal) / viewNormalDot;

			//// If the distance is less then zero; the triangle is not visible
			//if (t < ray.min || t > ray.max)
			//{
			//	return false;
			//}

			//// Calculate the hit point on the triangle
			//Vector3 hitPoint{ ray.origin + ray.direction * t };

			//// Make sure that the hit point is inside the triangle by checking its location compared to the edges
			////		If point is not in triangle, return false
			//if(!(IsToRightSideOfEdge(hitPoint, triangle.v0, triangle.v1, triangle.normal)
			//	&& IsToRightSideOfEdge(hitPoint, triangle.v1, triangle.v2, triangle.normal)
			//	&& IsToRightSideOfEdge(hitPoint, triangle.v2, triangle.v0, triangle.normal)))
			//{
			//	return false;
			//}

			//// If hit records needs to be ignored, just return true
			//if (ignoreHitRecord) return true;

			//hitRecord.t = t;
			//hitRecord.origin = hitPoint;
			//hitRecord.normal = triangle.normal;
			//hitRecord.materialIndex = triangle.materialIndex;
			//hitRecord.didHit = true;

			//return true;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion

#pragma region TriangeMesh HitTest
		inline bool SlabTest(const Ray& ray, const Vector3& minAABB, const Vector3& maxAABB)
		{
			const float tx1 = (minAABB.x - ray.origin.x) * ray.inversedDirection.x;
			const float tx2 = (maxAABB.x - ray.origin.x) * ray.inversedDirection.x;

			float tmin = std::min(tx1, tx2);
			float tmax = std::max(tx1, tx2);

			const float ty1 = (minAABB.y - ray.origin.y) * ray.inversedDirection.y;
			const float ty2 = (maxAABB.y - ray.origin.y) * ray.inversedDirection.y;

			tmin = std::max(tmin, std::min(ty1, ty2));
			tmax = std::min(tmax, std::max(ty1, ty2));

			const float tz1 = (minAABB.z - ray.origin.z) * ray.inversedDirection.z;
			const float tz2 = (maxAABB.z - ray.origin.z) * ray.inversedDirection.z;

			tmin = std::max(tmin, std::min(tz1, tz2));
			tmax = std::min(tmax, std::max(tz1, tz2));

			return tmax > 0 && tmax >= tmin;
		}

		inline void IntersectBVH(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool& hasHit, HitRecord& curClosestHit, bool ignoreHitRecord, unsigned int bvhNodeIdx)
		{
			BVHNode& node{ mesh.pBvhNodes[bvhNodeIdx] };

			// Slabtest
			if (!SlabTest(ray, node.aabbMin, node.aabbMax)) return;

			if (node.IsLeaf())
			{
				// Create a triangle object that is shared for all triangles
				Triangle triangle{};

				// Apply the mesh cullmode and material to the triangle
				triangle.cullMode = mesh.cullMode;
				triangle.materialIndex = mesh.materialIndex;

				// For each triangle
				for (unsigned int triangleIdx{}; triangleIdx < node.indicesCount; triangleIdx += 3)
				{
					// Set the position and normal of the current triangle to the triangle object
					triangle.v0 = mesh.transformedPositions[mesh.indices[node.firstIndice + triangleIdx]];
					triangle.v1 = mesh.transformedPositions[mesh.indices[node.firstIndice + triangleIdx + 1]];
					triangle.v2 = mesh.transformedPositions[mesh.indices[node.firstIndice + triangleIdx + 2]];
					triangle.normal = mesh.transformedNormals[(node.firstIndice + triangleIdx) / 3];

					// If the ray hits a triangle in the mesh, check if it is closer then the previous hit triangle
					if (HitTest_Triangle(triangle, ray, curClosestHit, ignoreHitRecord))
					{
						hasHit = true;

						// If the hit records needs to be ignored, it doesn't matter where the triangle is, so just return true
						if (ignoreHitRecord) return;

						// Check if the current hit is closer then the previous hit
						if (hitRecord.t > curClosestHit.t)
						{
							hitRecord = curClosestHit;
						}
					}
				}
			}
			else
			{
				IntersectBVH(mesh, ray, hitRecord, hasHit, curClosestHit, ignoreHitRecord, node.leftChild);
				IntersectBVH(mesh, ray, hitRecord, hasHit, curClosestHit, ignoreHitRecord, node.leftChild + 1);
			}
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//// Slabtest
			//if (!SlabTest(ray, mesh.transformedMinAABB, mesh.transformedMaxAABB))	return false;

			// Current closest hit
			HitRecord tempHit{};
			bool hasHit{};

			IntersectBVH(mesh, ray, hitRecord, hasHit, tempHit, ignoreHitRecord, 0);

			//// Create a triangle object that is shared for all triangles
			//Triangle triangle{};

			//// Apply the mesh cullmode and material to the triangle
			//triangle.cullMode = mesh.cullMode;
			//triangle.materialIndex = mesh.materialIndex;

			//// For each triangle
			//for (int triangleIdx{}; triangleIdx < mesh.indices.size(); triangleIdx += 3)
			//{
			//	// Set the position and normal of the current triangle to the triangle object
			//	triangle.v0 = mesh.transformedPositions[mesh.indices[triangleIdx]];
			//	triangle.v1 = mesh.transformedPositions[mesh.indices[triangleIdx + 1]];
			//	triangle.v2 = mesh.transformedPositions[mesh.indices[triangleIdx + 2]];
			//	triangle.normal = mesh.transformedNormals[triangleIdx / 3];

			//	// If the ray hits a triangle in the mesh, check if it is closer then the previous hit triangle
			//	if (HitTest_Triangle(triangle, ray, tempHit, ignoreHitRecord))
			//	{
			//		// If the hit records needs to be ignored, it doesn't matter where the triangle is, so just return true
			//		if (ignoreHitRecord) return true;

			//		// Check if the current hit is closer then the previous hit
			//		if (hitRecord.t > tempHit.t)
			//		{
			//			hitRecord = tempHit;
			//		}
			//		hasHit = true;
			//	}
			//}

			return hasHit;
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
			switch (light.type)
			{
			case LightType::Directional:
				return -light.direction;
			case LightType::Point:
				const Vector3 lightDirection{ light.origin - origin };
				return lightDirection;
			}

			// If no light type is matched; return a zero Vector3
			return Vector3{};
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			ColorRGB lightEnergy{};

			switch (light.type)
			{
			case LightType::Directional:
				lightEnergy = light.color * light.intensity;	// If light is directional, discard the distance from the light
				break;
			case LightType::Point:
			{
				Vector3 targetToLight{ GetDirectionToLight(light, target) };
				lightEnergy = light.color * light.intensity / targetToLight.SqrMagnitude();	// If light is point, divide the intensity 
			}
				break;
			}

			return lightEnergy;
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
					float i0{}, i1{}, i2{};
#if defined(MAYA_IMPORT)
					std::string s0, s1, s2;
					file >> s0 >> s1 >> s2;
					const char delimiter{ '/' };
					if (!(s0.size() > 0 && s1.size() > 0 && s2.size() > 0)) continue;
					i0 = std::stof(s0.substr(0, s0.find(delimiter)));
					i1 = std::stof(s1.substr(0, s1.find(delimiter)));
					i2 = std::stof(s2.substr(0, s2.find(delimiter)));
#else
					file >> i0 >> i1 >> i2;
#endif

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
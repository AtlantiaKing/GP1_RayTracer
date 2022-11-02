#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"

//#define MAYA_IMPORT
#define USE_MOLLERTRUMBORE
//#define USE_ANALYTIC_SPHERE

namespace dae
{
	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
#ifdef USE_ANALYTIC_SPHERE
			// Analytic

			// Calculate a vector from the origin of the ray to the sphere origin
			const Vector3 raySphereVector{ ray.origin - sphere.origin };

			// Calculate all parts of the quadratic formula
			const float a{ /*Vector3::Dot(ray.direction, ray.direction)*/ 1.0f };
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

			const float discriminantDivisor{ 1.0f / (2.0f * a)};

			// Calculate both hit point distances and use the smallest
			t = (-b - sqrtDiscriminant) * discriminantDivisor;
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
#else
			// Geometric

			// Calculate a vector from the origin of the ray to the sphere origin
			const Vector3 raySphereVector{ sphere.origin - ray.origin };

			// Calculate the distance from the ray to the sphere
			const float raySphereDistanceSqr{ raySphereVector.SqrMagnitude() };										// ||RaySphere||²

			// Project the ray sphere distance on the ray direction
			const float raySphereDistanceProjectedOnRay{ Vector3::Dot(raySphereVector, ray.direction) };		// ||RaySphere||*cos(angle)

			// Project the ray sphere distance perpendicular on the ray direction
			const float raySpherePerpendicularDistanceSqr{ raySphereDistanceSqr - Square(raySphereDistanceProjectedOnRay) };	// (1-cos(angle)²)*||raysphere||² = (sin(angle)*||raysphere||)²

			// If (the radius of the sphere sqrd - raySpherePerpendicularDistanceSqr) is smaller then 0, then you can't do sqrt and the ray is not intersecting with the sphere
			if (Square(sphere.radius) < raySpherePerpendicularDistanceSqr) return false;

			// Calculate the distance from the sphere origin to the hit point
			const float sphereHitPointDistance{ sqrtf(Square(sphere.radius) - raySpherePerpendicularDistanceSqr) };	// Pythagorean Theorem in a circle

			// Calculate the distance between the ray origin and the hit point
			float t = raySphereDistanceProjectedOnRay - sphereHitPointDistance;

			// If the hit point is further away then the accepted ray bounds, return false
			if (t < ray.min || t > ray.max) return false;

			// If no hitrecord needs to be calculated, return true
			if (ignoreHitRecord) return true;

			// Calculate the normal of the hit point
			const Vector3 hitPoint{ ray.origin + ray.direction * t };
			const Vector3 hitNormal{ sphere.origin, hitPoint };

			// Set the hit record to the calculated information
			hitRecord.didHit = true;
			hitRecord.materialIndex = sphere.materialIndex;
			hitRecord.origin = hitPoint;
			hitRecord.normal = hitNormal.Normalized();
			hitRecord.t = t;

			return true;
#endif
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
			if (t < ray.min || t > ray.max) return false;

			// If no hitrecord needs to be calculated, return true
			if (ignoreHitRecord) return true;

			// Set the hit record to the calculated information
			hitRecord.didHit = true;
			hitRecord.materialIndex = plane.materialIndex;
			hitRecord.origin = ray.origin + ray.direction * t;
			hitRecord.normal = plane.normal;
			hitRecord.t = t;

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
			// Calculate the current edge
			const Vector3 edge{ v1 - v0 };

			// Calculate the vector between the first vertex and the point
			const Vector3 startToPoint{ point - v0 };

			// Calculate cross product from edge to start to point
			const Vector3 edgePointCross{ Vector3::Cross(edge, startToPoint) };

			// The point is to the right side of the edge if the cross product and the normal of the triangle are in the same direction (cos(angle) > 0)
			return Vector3::Dot(edgePointCross, normal) > 0;
		}

		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			// Calculate the dot product with the triangle normal and the view direction
			const float viewNormalDot{ Vector3::Dot(ray.direction, triangle.normal) };

			// If the camera looks perpendicular on the normal, the triangle is not visible
			if (abs(viewNormalDot) < FLT_EPSILON) return false;

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

#ifdef USE_MOLLERTRUMBORE
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

			hitRecord.didHit = true;
			hitRecord.materialIndex = triangle.materialIndex;
			hitRecord.origin = ray.origin + ray.direction * t;
			hitRecord.normal = triangle.normal;
			hitRecord.t = t;

			return true;
#else
			// Calculate the center of the triangle
			Vector3 center{ (triangle.v0 + triangle.v1 + triangle.v2) / 3.0f };
			
			// Calculate a vector from the camera to the triangle
			const Vector3 planeRayDistance{ center - ray.origin };

			// Calculate the distance from ray hit
			const float t = Vector3::Dot(planeRayDistance, triangle.normal) / viewNormalDot;

			// If the distance is less then zero; the triangle is not visible
			if (t < ray.min || t > ray.max)
			{
				return false;
			}

			// Calculate the hit point on the triangle
			Vector3 hitPoint{ ray.origin + ray.direction * t };

			// Make sure that the hit point is inside the triangle by checking its location compared to the edges
			//		If point is not in triangle, return false
			if(!(IsToRightSideOfEdge(hitPoint, triangle.v0, triangle.v1, triangle.normal)
				&& IsToRightSideOfEdge(hitPoint, triangle.v1, triangle.v2, triangle.normal)
				&& IsToRightSideOfEdge(hitPoint, triangle.v2, triangle.v0, triangle.normal)))
			{
				return false;
			}

			// If hit records needs to be ignored, just return true
			if (ignoreHitRecord) return true;

			hitRecord.normal = triangle.normal;
			hitRecord.origin = ray.origin + ray.direction * t;
			hitRecord.didHit = true;
			hitRecord.materialIndex = triangle.materialIndex;
			hitRecord.t = t;

			return true;
#endif
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

		inline void IntersectBVH(const TriangleMesh& mesh, const Ray& ray, Triangle& sharedTriangle, HitRecord& hitRecord, bool& hasHit, HitRecord& curClosestHit, bool ignoreHitRecord, unsigned int bvhNodeIdx)
		{
			// Get the current node
			BVHNode& node{ mesh.pBvhNodes[bvhNodeIdx] };

			// Slabtest
			if (!SlabTest(ray, node.aabbMin, node.aabbMax)) return;

			// If the current node is not the end node, recursively search the two child nodes 
			if (!node.IsLeaf())
			{
				IntersectBVH(mesh, ray, sharedTriangle, hitRecord, hasHit, curClosestHit, ignoreHitRecord, node.leftChild);
				IntersectBVH(mesh, ray, sharedTriangle, hitRecord, hasHit, curClosestHit, ignoreHitRecord, node.leftChild + 1);
				return;
			}

			// For each triangle in the node
			for (unsigned int triangleIdx{}; triangleIdx < node.indicesCount; triangleIdx += 3)
			{
				// Set the position and normal of the current triangle to the triangle object
				sharedTriangle.v0 = mesh.transformedPositions[mesh.indices[node.firstIndice + triangleIdx]];
				sharedTriangle.v1 = mesh.transformedPositions[mesh.indices[node.firstIndice + triangleIdx + 1]];
				sharedTriangle.v2 = mesh.transformedPositions[mesh.indices[node.firstIndice + triangleIdx + 2]];
				sharedTriangle.normal = mesh.transformedNormals[(node.firstIndice + triangleIdx) / 3];

				// If the ray doesn't a triangle in the mesh, continue to the next triangle
				if (!HitTest_Triangle(sharedTriangle, ray, curClosestHit, ignoreHitRecord)) continue;

				// If the ray hits a triangle, set hasHit to true
				hasHit = true;

				// If the hit records needs to be ignored, it doesn't matter if there is a triangle closer or not, so just return
				if (ignoreHitRecord) return;

				// Check if the current hit is closer then the previous hit
				if (hitRecord.t > curClosestHit.t)
				{
					hitRecord = curClosestHit;
				}
			}
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
#ifndef USE_BVH
			// Slabtest
			if (!SlabTest(ray, mesh.transformedMinAABB, mesh.transformedMaxAABB)) return false;
#endif
			// Current closest hit
			HitRecord tempHit{};
			bool hasHit{};

			// Create a triangle object that is shared for all triangles
			Triangle tempTriangle{};

			// Apply the mesh cullmode and material to the triangle
			tempTriangle.cullMode = mesh.cullMode;
			tempTriangle.materialIndex = mesh.materialIndex;

#ifdef USE_BVH
			// Search the BVH for the triangles closest to the ray and do hit tests with these
			IntersectBVH(mesh, ray, tempTriangle, hitRecord, hasHit, tempHit, ignoreHitRecord, 0);
#else
			// For each triangle
			for (int triangleIdx{}; triangleIdx < mesh.indices.size(); triangleIdx += 3)
			{
				// Set the position and normal of the current triangle to the triangle object
				tempTriangle.v0 = mesh.transformedPositions[mesh.indices[triangleIdx]];
				tempTriangle.v1 = mesh.transformedPositions[mesh.indices[triangleIdx + 1]];
				tempTriangle.v2 = mesh.transformedPositions[mesh.indices[triangleIdx + 2]];
				tempTriangle.normal = mesh.transformedNormals[triangleIdx / 3];

				// If the ray doesn't a triangle in the mesh, continue to the next triangle
				if (!HitTest_Triangle(tempTriangle, ray, tempHit, ignoreHitRecord)) continue;

				// If the hit records needs to be ignored, it doesn't matter where the triangle is, so just return true
				if (ignoreHitRecord) return true;

				// Check if the current hit is closer then the previous hit
				if (hitRecord.t > tempHit.t)
				{
					hitRecord = tempHit;
				}

				// If the ray hits a triangle, set hasHit to true
				hasHit = true;
			}
#endif

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
				return light.origin - origin;
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
					float i0, i1, i2;
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
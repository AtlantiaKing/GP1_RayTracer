#pragma once
#include <cassert>
#include <nmmintrin.h>

#include "Math.h"
#include "vector"

//#define USE_BVH

namespace dae
{
#pragma region GEOMETRY
	struct Sphere
	{
		Vector3 origin{};
		float radius{};

		unsigned char materialIndex{ 0 };
	};

	struct Plane
	{
		Vector3 origin{};
		Vector3 normal{};

		unsigned char materialIndex{ 0 };
	};

	enum class TriangleCullMode
	{
		FrontFaceCulling,
		BackFaceCulling,
		NoCulling
	};

	struct Triangle
	{
		Triangle() = default;
		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2, const Vector3& _normal) :
			v0{ _v0 }, v1{ _v1 }, v2{ _v2 }, normal{ _normal.Normalized() } {}

		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2) :
			v0{ _v0 }, v1{ _v1 }, v2{ _v2 }
		{
			const Vector3 edgeV0V1 = v1 - v0;
			const Vector3 edgeV0V2 = v2 - v0;
			normal = Vector3::Cross(edgeV0V1, edgeV0V2).Normalized();
		}

		Vector3 v0{};
		Vector3 v1{};
		Vector3 v2{};

		Vector3 normal{};

		TriangleCullMode cullMode{};
		unsigned char materialIndex{};
	};

	struct BVHNode
	{
		Vector3 aabbMin{};
		Vector3 aabbMax{};
		size_t leftChild{};
		size_t firstIndice{};
		size_t indicesCount{};
		bool IsLeaf() { return indicesCount > 0; };
	};

	struct AABB
	{
		Vector3 min{ Vector3::One * FLT_MAX };
		Vector3 max{ Vector3::One * FLT_MIN };
		void Grow(const Vector3& point)
		{
			min = Vector3::Min(min, point);
			max = Vector3::Max(max, point);
		}
		void Grow(const AABB& bounds)
		{
			min = Vector3::Min(min, bounds.min);
			max = Vector3::Max(max, bounds.max);
		}
		float GetArea()
		{
			Vector3 boxSize{ max - min };
			return boxSize.x * boxSize.y + boxSize.y * boxSize.z + boxSize.z * boxSize.x;
		}
	};

	struct Bin
	{
		AABB bounds{};
		int indicesCount{};
	};

	struct TriangleMesh
	{
		TriangleMesh() = default;
		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, TriangleCullMode _cullMode):
		positions(_positions), indices(_indices), cullMode(_cullMode)
		{
			//Calculate Normals
			CalculateNormals();

			//Update Transforms
			UpdateTransforms();
		}

		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, const std::vector<Vector3>& _normals, TriangleCullMode _cullMode) :
			positions(_positions), indices(_indices), normals(_normals), cullMode(_cullMode)
		{
			UpdateTransforms();
		}

		~TriangleMesh()
		{
			delete[] pBvhNodes;
		}

		std::vector<Vector3> positions{};
		std::vector<Vector3> normals{};
		std::vector<int> indices{};
		unsigned char materialIndex{};

		TriangleCullMode cullMode{TriangleCullMode::BackFaceCulling};

		Matrix rotationTransform{};
		Matrix translationTransform{};
		Matrix scaleTransform{};

		Vector3 minAABB;
		Vector3 maxAABB;

		Vector3 transformedMinAABB;
		Vector3 transformedMaxAABB;

		std::vector<Vector3> transformedPositions{};
		std::vector<Vector3> transformedNormals{};

		BVHNode* pBvhNodes{};
		unsigned int rootBvhNodeIdx{};
		unsigned int bvhNodesUsed{};

		void Translate(const Vector3& translation)
		{
			translationTransform = Matrix::CreateTranslation(translation);
		}

		void RotateY(float yaw)
		{
			rotationTransform = Matrix::CreateRotationY(yaw);
		}

		void Scale(const Vector3& scale)
		{
			scaleTransform = Matrix::CreateScale(scale);
		}

		void AppendTriangle(const Triangle& triangle, bool ignoreTransformUpdate = false)
		{
			int startIndex = static_cast<int>(positions.size());

			positions.push_back(triangle.v0);
			positions.push_back(triangle.v1);
			positions.push_back(triangle.v2);

			indices.push_back(startIndex);
			indices.push_back(++startIndex);
			indices.push_back(++startIndex);

			normals.push_back(triangle.normal);

			//Not ideal, but making sure all vertices are updated
			if(!ignoreTransformUpdate)
				UpdateTransforms();
		}

		void CalculateNormals()
		{
			normals.clear();
			normals.reserve(indices.size() / 3);

			for (size_t triangle{}; triangle < indices.size(); triangle += 3)
			{
				Vector3& v0 = positions[indices[triangle]];
				Vector3& v1 = positions[indices[triangle + 1]];
				Vector3& v2 = positions[indices[triangle + 2]];

				Vector3 edge0 = v1 - v0;
				Vector3 edge1 = v2 - v0;

				normals.emplace_back(Vector3::Cross(edge0, edge1).Normalized());
			}
		}

		void UpdateAABB()
		{
			if (positions.size() > 0)
			{
				minAABB = positions[0];
				maxAABB = positions[0];
				for (const Vector3& p : positions)
				{
					minAABB = Vector3::Min(p, minAABB);
					maxAABB = Vector3::Max(p, maxAABB);
				}
			}
		}

		void UpdateTransforms()
		{
			const Matrix finalTranformation{ scaleTransform * rotationTransform * translationTransform };
			//const Matrix normalTranformation{ rotationTransform * translationTransform };

			transformedPositions.clear();
			transformedPositions.reserve(positions.size());
			for (const Vector3& position : positions)
			{
				transformedPositions.emplace_back(finalTranformation.TransformPoint(position));
			}

			transformedNormals.clear();
			transformedNormals.reserve(normals.size());
			for (const Vector3& normal : normals)
			{
				transformedNormals.emplace_back(finalTranformation.TransformVector(normal).Normalized());
			}

#ifdef USE_BVH
			UpdateBVH();
#else
			UpdateTransformedAABB(finalTranformation);
#endif
		}

		void UpdateTransformedAABB(const Matrix& finalTransform)
		{
			// AABB Update: be careful -> transform the 8 vertices of the aabb
			// and calculate the new min and max
			Vector3 tMinAABB = finalTransform.TransformPoint(minAABB);
			Vector3 tMaxAABB = tMinAABB;
			// (xmax, ymin, zmin)
			Vector3 tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmax, ymin, zmax)
			tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmin, ymin, zmax)
			tAABB = finalTransform.TransformPoint(minAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (minx, ymax, zmin)
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmax, ymax, zmin)
			tAABB = finalTransform.TransformPoint(maxAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmax, ymax, zmax)
			tAABB = finalTransform.TransformPoint(maxAABB);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmin, ymax, zmax)
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

			transformedMinAABB = tMinAABB;
			transformedMaxAABB = tMaxAABB;
		}

		void UpdateBVH()
		{
			// The max amount of nodes needed is (nrTriangles * 2 - 1)
			if(!pBvhNodes) pBvhNodes = new BVHNode[indices.size() / 3 * 2 - 1]{};

			bvhNodesUsed = 0;

			BVHNode& root = pBvhNodes[rootBvhNodeIdx];
			root.leftChild = 0;
			root.firstIndice = 0;
			root.indicesCount = static_cast<unsigned int>(indices.size());

			UpdateBVHNodeBounds(rootBvhNodeIdx);

			Subdivide(rootBvhNodeIdx);
		}

		inline void UpdateBVHNodeBounds(int nodeIdx)
		{
			BVHNode& node{ pBvhNodes[nodeIdx] };

			node.aabbMin = Vector3::One * FLT_MAX;
			node.aabbMax = Vector3::One * FLT_MIN;

			for (size_t i{ node.firstIndice }; i < node.firstIndice + node.indicesCount; ++i)
			{
				Vector3& curVertex{ transformedPositions[indices[i]] };
				node.aabbMin = Vector3::Min(node.aabbMin, curVertex);
				node.aabbMax = Vector3::Max(node.aabbMax, curVertex);
			}
		}

		inline void Subdivide(int nodeIdx)
		{
			BVHNode& node{ pBvhNodes[nodeIdx] };

			const int maxNrTrianglesPerNode{ 2 };

			if (node.indicesCount <= maxNrTrianglesPerNode * 3) return;

			// Determine split axis and position using SAH
			int axis{ -1 };
			float splitPos{ 0 };
			float cost{ FindBestSplitPlane(node, axis, splitPos) };
			
			const float noSplitCost{ CalculateNodeCost(node) };
			if (cost >= noSplitCost) return;

			// in-place partition
			size_t i{ node.firstIndice };
			size_t j{ i + node.indicesCount - 1 };
			while (i <= j)
			{
				const Vector3 centroid{ (transformedPositions[indices[i]] + transformedPositions[indices[i + 1]] + transformedPositions[indices[i + 2]]) / 3.0f };

				if (centroid[axis] < splitPos)
				{
					i += 3;
				}
				else
				{
					std::swap(indices[i], indices[j - 2]);
					std::swap(indices[i + 1], indices[j - 1]);
					std::swap(indices[i + 2], indices[j]);
					std::swap(normals[i / 3], normals[(j - 2) / 3]);
					std::swap(transformedNormals[i / 3], transformedNormals[(j - 2) / 3]);

					j -= 3;
				}
			}

			// abort split if one of the sides is empty
			size_t leftCount{ i - node.firstIndice };
			if (leftCount == 0 || leftCount == node.indicesCount)
			{
				return;
			}

			// Create child nodes
			unsigned int leftChildIdx{ ++bvhNodesUsed };
			unsigned int rightChildIdx{ ++bvhNodesUsed };

			node.leftChild = leftChildIdx;

			pBvhNodes[leftChildIdx].firstIndice = node.firstIndice;
			pBvhNodes[leftChildIdx].indicesCount = leftCount;
			pBvhNodes[rightChildIdx].firstIndice = i;
			pBvhNodes[rightChildIdx].indicesCount = node.indicesCount - leftCount;
			node.indicesCount = 0;

			UpdateBVHNodeBounds(leftChildIdx);
			UpdateBVHNodeBounds(rightChildIdx);

			// Recurse
			Subdivide(leftChildIdx);
			Subdivide(rightChildIdx);
		}

		inline float FindBestSplitPlane(const BVHNode& node, int& axis, float& splitPos) const
		{
			float bestCost{ FLT_MAX };
			for (int curAxis{}; curAxis < 3; curAxis++)
			{
				float boundsMin{ FLT_MAX };
				float boundsMax{ FLT_MIN };
				for (unsigned int i{}; i < node.indicesCount; i += 3)
				{
					const Vector3 centroid{ (transformedPositions[indices[node.firstIndice + i]] + transformedPositions[indices[node.firstIndice + i + 1]] + transformedPositions[indices[node.firstIndice + i + 2]]) / 3.0f };
					boundsMin = std::min(centroid[curAxis], boundsMin);
					boundsMax = std::max(centroid[curAxis], boundsMax);
				}

				if (abs(boundsMin - boundsMax) < FLT_EPSILON) continue;

				const int nrBins{ 8 };

				Bin bins[nrBins];

				float scale{ nrBins / (boundsMax - boundsMin) };

				for (unsigned int i{}; i < node.indicesCount; i += 3)
				{
					const Vector3& v0{ transformedPositions[indices[node.firstIndice + i]] };
					const Vector3& v1{ transformedPositions[indices[node.firstIndice + i + 1]] };
					const Vector3& v2{ transformedPositions[indices[node.firstIndice + i + 2]] };

					const Vector3 centroid{ (v0 + v1 + v2) / 3.0f }; 

					int binIdx{ std::min(nrBins - 1, static_cast<int>((centroid[curAxis] - boundsMin) * scale)) };

					bins[binIdx].indicesCount += 3;
					bins[binIdx].bounds.Grow(v0);
					bins[binIdx].bounds.Grow(v1);
					bins[binIdx].bounds.Grow(v2);
				}

				float leftArea[nrBins - 1]{};
				float rightArea[nrBins - 1]{};
				float leftCount[nrBins - 1]{};
				float rightCount[nrBins - 1]{};

				AABB leftBox;
				AABB rightBox;
				float leftSum{};
				float rightSum{};

				for (int i{}; i < nrBins - 1; ++i)
				{
					leftSum += bins[i].indicesCount;
					leftCount[i] = leftSum;
					leftBox.Grow(bins[i].bounds);
					leftArea[i] = leftBox.GetArea();

					rightSum += bins[nrBins - 1 - i].indicesCount;
					rightCount[nrBins - 2 - i] = rightSum;
					rightBox.Grow(bins[nrBins - 1 - i].bounds);
					rightArea[nrBins - 2 - i] = rightBox.GetArea();
				}

				scale = (boundsMax - boundsMin) / nrBins;

				for (unsigned int i{}; i < nrBins - 1; ++i)
				{
					const float planeCost{ leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i] };

					if (planeCost < bestCost)
					{
						splitPos = boundsMin + scale * (i + 1);
						axis = curAxis;
						bestCost = planeCost;
					}
				}
			}
			return bestCost;
		}

		inline float CalculateNodeCost(const BVHNode& node) const
		{
			const Vector3 boxSize{ node.aabbMax - node.aabbMin };
			const float parentArea{ boxSize.x * boxSize.y + boxSize.y * boxSize.z + boxSize.z * boxSize.x };
			return node.indicesCount * parentArea;
		}

		inline float EvaluateSAH(BVHNode& node, int axis, float pos) const
		{
			AABB leftBox{};
			AABB rightBox{};
			int leftCount{};
			int rightCount{};

			for (unsigned int i{}; i < node.indicesCount; i += 3)
			{
				const Vector3& v0{ transformedPositions[indices[node.firstIndice + i]] };
				const Vector3& v1{ transformedPositions[indices[node.firstIndice + i + 1]] };
				const Vector3& v2{ transformedPositions[indices[node.firstIndice + i + 2]] };

				const Vector3 centroid{ (v0 + v1 + v2) / 3.0f };

				if (centroid[axis] < pos)
				{
					++leftCount;
					leftBox.Grow(v0);
					leftBox.Grow(v1);
					leftBox.Grow(v2);
				}
				else
				{
					++rightCount;
					rightBox.Grow(v0);
					rightBox.Grow(v1);
					rightBox.Grow(v2);
				}
			}
			float cost{ leftCount * leftBox.GetArea() + rightCount * rightBox.GetArea() };
			return cost > 0 ? cost : FLT_MAX;
		}
	};
#pragma endregion
#pragma region LIGHT
	enum class LightType
	{
		Point,
		Directional
	};

	struct Light
	{
		Vector3 origin{};
		Vector3 direction{};
		ColorRGB color{};
		float intensity{};
		float radius{};

		LightType type{};
	};
#pragma endregion
#pragma region MISC
	struct Ray
	{
		Ray(const Vector3& _origin, const Vector3& _direction)
			: origin{ _origin }
			, direction{ _direction }
			, inversedDirection{ Vector3{ 1.0f / _direction.x,  1.0f / _direction.y,  1.0f / _direction.z } }
		{

		}
		Ray(const Vector3& _origin, const Vector3& _direction, float _min, float _max)
			: origin{ _origin }
			, direction{ _direction }
			, inversedDirection{ Vector3{ 1.0f / _direction.x,  1.0f / _direction.y,  1.0f / _direction.z } }
			, min{ _min }
			, max{ _max }
		{

		}

		Vector3 origin{};
		Vector3 direction{};
		Vector3 inversedDirection{};

		float min{ 0.0001f };
		float max{ FLT_MAX };
	};

	struct HitRecord
	{
		Vector3 origin{};
		Vector3 normal{};
		float t = FLT_MAX;

		bool didHit{ false };
		unsigned char materialIndex{ 0 };
	};
#pragma endregion
}
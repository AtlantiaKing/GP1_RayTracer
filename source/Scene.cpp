#include "Scene.h"
#include "Utils.h"
#include "Material.h"

namespace dae {

#pragma region Base Scene
	//Initialize Scene with Default Solid Color Material (RED)
	Scene::Scene():
		m_Materials({ new Material_SolidColor({1,0,0})})
	{
		m_SphereGeometries.reserve(32);
		m_PlaneGeometries.reserve(32);
		m_TriangleMeshGeometries.reserve(32);
		m_Lights.reserve(32);
	}

	Scene::~Scene()
	{
		for(auto& pMaterial : m_Materials)
		{
			delete pMaterial;
			pMaterial = nullptr;
		}

		m_Materials.clear();
	}

	void dae::Scene::GetClosestHit(const Ray& ray, HitRecord& closestHit) const
	{
		HitRecord tempHit{ };

		// For each sphere
		for (const Sphere& sphere : m_SphereGeometries)
		{
			// If the ray hits a sphere, check if the sphere is closer then the previous hit, else continue
			if (GeometryUtils::HitTest_Sphere(sphere, ray, tempHit))
			{
				// If the current hit point is closer that the previous hit point, overwrite
				if (closestHit.t > tempHit.t)
				{
					closestHit.didHit = tempHit.didHit;
					closestHit.materialIndex = tempHit.materialIndex;
					closestHit.origin = tempHit.origin;
					closestHit.normal = tempHit.normal;
					closestHit.t = tempHit.t;
				}
			}
		}

		// For each plane
		for (const Plane& plane : m_PlaneGeometries)
		{
			// If the ray hits a plane, check if the plane is closer then the previous hit, else continue
			if (GeometryUtils::HitTest_Plane(plane, ray, tempHit))
			{
				// If nothing has been hit yet, set closestHit to the current tempHit
				// Else only set closestHit if the current hit point is closer that the previous hit point
				if (closestHit.t > tempHit.t)
				{
					closestHit.didHit = tempHit.didHit;
					closestHit.materialIndex = tempHit.materialIndex;
					closestHit.origin = tempHit.origin;
					closestHit.normal = tempHit.normal;
					closestHit.t = tempHit.t;
				}
			}
		}
		
		// For each triangle mesh
		for (const TriangleMesh& triangleMesh : m_TriangleMeshGeometries)
		{
			// If the ray hits a triangle mesh, check if the triangle mesh is closer then the previous hit, else continue
			if (GeometryUtils::HitTest_TriangleMesh(triangleMesh, ray, tempHit))
			{
				// If nothing has been hit yet, set closestHit to the current tempHit
				// Else only set closestHit if the current hit point is closer that the previous hit point
				if (closestHit.t > tempHit.t)
				{
					closestHit.didHit = tempHit.didHit;
					closestHit.materialIndex = tempHit.materialIndex;
					closestHit.origin = tempHit.origin;
					closestHit.normal = tempHit.normal;
					closestHit.t = tempHit.t;
				}
			}
		}

		// For each triangle
		//for (const Triangle& triangle : m_Triangles)
		//{
		//	// If the ray hits a triangle, check if the triangle is closer then the previous hit, else continue
		//	HitRecord tempHit{ };
		//	if (GeometryUtils::HitTest_Triangle(triangle, ray, tempHit))
		//	{
		//		// If nothing has been hit yet, set closestHit to the current tempHit
		//		// Else only set closestHit if the current hit point is closer that the previous hit point
		//		if (closestHit.t > tempHit.t)
		//		{
		//			closestHit = tempHit;
		//		}
		//	}
		//}
	}

	bool Scene::DoesHit(const Ray& ray) const
	{
		// For each sphere
		for (const Sphere& sphere : m_SphereGeometries)
		{
			// If the ray hits a sphere, stop the function and return true
			if (GeometryUtils::HitTest_Sphere(sphere, ray))
			{
				return true;
			}
		}

		// For each plane
		for (const Plane& plane : m_PlaneGeometries)
		{
			// If the ray hits a plane, stop the function and return true
			if (GeometryUtils::HitTest_Plane(plane, ray))
			{
				return true;
			}
		}

		// For each triangle mesh
		for (const TriangleMesh& triangleMesh : m_TriangleMeshGeometries)
		{
			// If the ray hits a triangle mesh, stop the function and return true
			if (GeometryUtils::HitTest_TriangleMesh(triangleMesh, ray))
			{
				return true;
			}
		}

		// For each triangle
		//for (const Triangle& triangle : m_Triangles)
		//{
		//	// If the ray hits a triangle, stop the function and return true
		//	if (GeometryUtils::HitTest_Triangle(triangle, ray))
		//	{
		//		return true;
		//	}
		//}

		return false;
	}

#pragma region Scene Helpers
	Sphere* Scene::AddSphere(const Vector3& origin, float radius, unsigned char materialIndex)
	{
		Sphere s;
		s.origin = origin;
		s.radius = radius;
		s.materialIndex = materialIndex;

		m_SphereGeometries.emplace_back(s);
		return &m_SphereGeometries.back();
	}

	Plane* Scene::AddPlane(const Vector3& origin, const Vector3& normal, unsigned char materialIndex)
	{
		Plane p;
		p.origin = origin;
		p.normal = normal;
		p.materialIndex = materialIndex;

		m_PlaneGeometries.emplace_back(p);
		return &m_PlaneGeometries.back();
	}

	TriangleMesh* Scene::AddTriangleMesh(TriangleCullMode cullMode, unsigned char materialIndex)
	{
		TriangleMesh m{};
		m.cullMode = cullMode;
		m.materialIndex = materialIndex;

		m_TriangleMeshGeometries.emplace_back(m);
		return &m_TriangleMeshGeometries.back();
	}

	Light* Scene::AddPointLight(const Vector3& origin, float intensity, const ColorRGB& color)
	{
		Light l;
		l.origin = origin;
		l.intensity = intensity;
		l.color = color;
		l.type = LightType::Point;

		m_Lights.emplace_back(l);
		return &m_Lights.back();
	}

	Light* Scene::AddDirectionalLight(const Vector3& direction, float intensity, const ColorRGB& color)
	{
		Light l;
		l.direction = direction;
		l.intensity = intensity;
		l.color = color;
		l.type = LightType::Directional;

		m_Lights.emplace_back(l);
		return &m_Lights.back();
	}

	unsigned char Scene::AddMaterial(Material* pMaterial)
	{
		m_Materials.push_back(pMaterial);
		return static_cast<unsigned char>(m_Materials.size() - 1);
	}
#pragma endregion
#pragma endregion

#pragma region SCENE W1
	void Scene_W1::Initialize()
	{
		//default: Material id0 >> SolidColor Material (RED)
		constexpr unsigned char matId_Solid_Red = 0;
		const unsigned char matId_Solid_Blue = AddMaterial(new Material_SolidColor{ colors::Blue });

		const unsigned char matId_Solid_Yellow = AddMaterial(new Material_SolidColor{ colors::Yellow });
		const unsigned char matId_Solid_Green = AddMaterial(new Material_SolidColor{ colors::Green });
		const unsigned char matId_Solid_Magenta = AddMaterial(new Material_SolidColor{ colors::Magenta });

		//Spheres
		AddSphere({ -25.f, 0.f, 100.f }, 50.f, matId_Solid_Red);
		AddSphere({ 25.f, 0.f, 100.f }, 50.f, matId_Solid_Blue);

		//Plane
		AddPlane({ -75.f, 0.f, 0.f }, { 1.f, 0.f,0.f }, matId_Solid_Green);
		AddPlane({ 75.f, 0.f, 0.f }, { -1.f, 0.f,0.f }, matId_Solid_Green);
		AddPlane({ 0.f, -75.f, 0.f }, { 0.f, 1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0.f, 75.f, 0.f }, { 0.f, -1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0.f, 0.f, 125.f }, { 0.f, 0.f,-1.f }, matId_Solid_Magenta);
	}
#pragma endregion
#pragma region SCENE W2
	void Scene_W2::Initialize()
	{
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.SetFovAngle(45.0f);

		//default:: Material id0 >> SolidColor Material (RED)
		constexpr unsigned char matId_Solid_Red = 0;
		const unsigned char matId_Solid_Blue = AddMaterial(new Material_SolidColor{ colors::Blue });

		const unsigned char matId_Solid_Yellow = AddMaterial(new Material_SolidColor{ colors::Yellow });
		const unsigned char matId_Solid_Green = AddMaterial(new Material_SolidColor{ colors::Green });
		const unsigned char matId_Solid_Magenta = AddMaterial(new Material_SolidColor{ colors::Magenta });

		//Plane
		AddPlane({ -5.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, matId_Solid_Green);
		AddPlane({ 5.0f, 0.0f, 0.0f }, { -1.0f, 0.0f, 0.0f }, matId_Solid_Green);
		AddPlane({ 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, matId_Solid_Yellow);
		AddPlane({ 0.0f, 10.0f, 0.0f }, { 0.0f, -1.0f, 0.0f }, matId_Solid_Yellow);
		AddPlane({ 0.0f, 0.0f, 10.0f }, { 0.0f, 0.0f,-1.0f }, matId_Solid_Magenta);

		//Spheres
		AddSphere({ -1.75f, 1.0f, 0.0f }, 0.75f, matId_Solid_Red);
		AddSphere({ 0.0f, 1.0f, 0.0f }, 0.75f, matId_Solid_Blue);
		AddSphere({ 1.75f, 1.0f, 0.0f }, 0.75f, matId_Solid_Red);
		AddSphere({ -1.75f, 3.0f, 0.0f }, 0.75f, matId_Solid_Blue);
		AddSphere({ 0.f, 3.0f, 0.0f }, 0.75f, matId_Solid_Red);
		AddSphere({ 1.75f, 3.0f, 0.0f }, 0.75f, matId_Solid_Blue);

		//Light
		AddPointLight({ 0.0f, 5.0f, -5.0f }, 70.0f, colors::White);
	}
#pragma endregion
	void Scene_W3::Initialize()
	{
		/*m_Camera.origin = { 0.f, 1.0f, -5.0f };
		m_Camera.SetFovAngle(45.0f);

		const auto matLambert_Red{ AddMaterial(new Material_Lambert(colors::Red, 1.0f)) };
		const auto matLambertPhong_Blue{ AddMaterial(new Material_LambertPhong(colors::Blue, 0.5f, 0.5f, 60.0f)) };
		const auto matLambert_Yellow{ AddMaterial(new Material_Lambert(colors::Yellow, 1.0f)) };

		AddSphere({ -0.75f, 1.0f, 0.0f }, 1.0f, matLambert_Red);
		AddSphere({ 0.75f, 1.0f, 0.0f }, 1.0f, matLambertPhong_Blue);

		AddPlane({ 0.0f, 0.0f, 0.0f }, { 0.0f,1.0f, 0.0f }, matLambert_Yellow);

		AddPointLight({ 0.0f, 5.0f, 5.0f }, 25.0f, colors::White);

		AddPointLight({ 0.0f, 2.5f, -5.0f }, 25.0f, colors::White);*/


		m_Camera.origin = { 0.0f, 3.0f, -9.0f };
		m_Camera.SetFovAngle(45.0f);

		const auto matCT_GrayRoughMetal = AddMaterial(new Material_CookTorrence({ 0.972f, 0.960f, 0.915f }, 1.0f, 1.0f));
		const auto matCT_GrayMediumMetal = AddMaterial(new Material_CookTorrence({ 0.972f, 0.960f, 0.915f }, 1.0f, 0.6f));
		const auto matCT_GraySmoothMetal = AddMaterial(new Material_CookTorrence({ 0.972f, 0.960f, 0.915f }, 1.0f, 0.1f));
		const auto matCT_GrayRoughPlastic = AddMaterial(new Material_CookTorrence({ 0.75f, 0.75f, 0.75f }, 0.0f, 1.f));
		const auto matCT_GrayMediumPlastic = AddMaterial(new Material_CookTorrence({ 0.75f, 0.75f, 0.75f }, 0.0f, 0.6f));
		const auto matCT_GraySmoothPlastic = AddMaterial(new Material_CookTorrence({ 0.75f, 0.75f, 0.75f }, 0.0f, 0.1f));

		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert({ 0.49f, 0.57f, 0.57f }, 1.0f));

		//Plane
		AddPlane(Vector3{ 0.0f, 0.0f, 10.0f }, Vector3{ 0.0f, 0.0f, -1.0f }, matLambert_GrayBlue);; //Back
		AddPlane(Vector3{ 0.0f, 0.0f, 0.0f }, Vector3{ 0.0f, 1.0f, 0.0f }, matLambert_GrayBlue);; //Bottom
		AddPlane(Vector3{ 0.0f, 10.0f, 0.0f }, Vector3{ 0.0f, -1.0f, 0.0f }, matLambert_GrayBlue);; //Top
		AddPlane(Vector3{ 5.0f, 0.f, 0.0f }, Vector3{ -1.0f, 0.0f, 0.0f }, matLambert_GrayBlue);; //Right
		AddPlane(Vector3{ -5.0f, 0.0f, 0.0f }, Vector3{ 1.0f, 0.0f, 0.0f }, matLambert_GrayBlue);; //Left

		// Temporary Lambert-Phong Spheres & Materials
		/*const auto matLambertPhong1{ AddMaterial(new Material_LambertPhong(colors::Blue, 0.5f, 0.5f, 3.0f)) };
		const auto matLambertPhong2{ AddMaterial(new Material_LambertPhong(colors::Blue, 0.5f, 0.5f, 15.0f)) };
		const auto matLambertPhong3{ AddMaterial(new Material_LambertPhong(colors::Blue, 0.5f, 0.5f, 30.0f)) };
		
		AddSphere(Vector3{ -1.75f, 1.0f, 0.0f }, 0.75f, matLambertPhong1);
		AddSphere(Vector3{ -0.0f, 1.0f, 0.0f }, 0.75f, matLambertPhong2);
		AddSphere(Vector3{ 1.75f, 1.0f, 0.0f }, 0.75f, matLambertPhong3);*/

		//Spheres
		AddSphere(Vector3{ -1.75f, 1.0f, 0.0f }, 0.75f, matCT_GrayRoughMetal);
		AddSphere(Vector3{ 0.0f, 1.0f, 0.0f }, 0.75f, matCT_GrayMediumMetal);
		AddSphere(Vector3{ 1.75f, 1.0f, 0.0f }, 0.75f, matCT_GraySmoothMetal);
		AddSphere(Vector3{ -1.75f, 3.0f, 0.0f }, 0.75f, matCT_GrayRoughPlastic);
		AddSphere(Vector3{ 0.0f, 3.0f, 0.0f }, 0.75f, matCT_GrayMediumPlastic);
		AddSphere(Vector3{ 1.75f, 3.0f, 0.0f }, 0.75f, matCT_GraySmoothPlastic);

		//Light
		AddPointLight(Vector3{ 0.0f, 5.0f, 5.0f }, 50.f, ColorRGB{ 1.0f, 0.61f, 0.45f }); // Backlight
		AddPointLight(Vector3{ -2.5f, 5.0f, -5.0f }, 70.f, ColorRGB{ 1.0f, 0.8f, 0.45f }); // Frontlight left
		AddPointLight(Vector3{ 2.5f, 2.5f, -5.0f}, 50.f, ColorRGB{ 0.34f, 0.47f, 0.68f});
	}

	void Scene_W4_TestScene::Initialize()
	{
		m_Camera.origin = { 0.0f, 1.0f, -5.0f };
		m_Camera.SetFovAngle(45.0f);

		//Materials
		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert({ 0.49f, 0.57f, 0.57f }, 1.f));
		const auto matLambert_White = AddMaterial(new Material_Lambert(colors::White, 1.f));

		//Plane
		AddPlane(Vector3{ 0.f, 0.f, 10.f }, Vector3{ 0.f, 0.f, -1.f }, matLambert_GrayBlue);; //Back
		AddPlane(Vector3{ 0.f, 0.f, 0.f }, Vector3{ 0.f, 1.f, 0.f }, matLambert_GrayBlue);; //Bottom
		AddPlane(Vector3{ 0.f, 10.f, 0.f }, Vector3{ 0.f, -1.f, 0.f }, matLambert_GrayBlue);; //Top
		AddPlane(Vector3{ 5.f, 0.f, 0.f }, Vector3{ -1.f, 0.f, 0.f }, matLambert_GrayBlue);; //Right
		AddPlane(Vector3{ -5.f, 0.f, 0.f }, Vector3{ 1.f, 0.f, 0.f }, matLambert_GrayBlue);; //Left


		////Triangle (Temp)
		//auto triangle = Triangle{ {-.75, .5f, 0.f},  {-0.75f, 2.0f, 0.f}, {.75f, .5f, 0.f} };
		//triangle.cullMode = TriangleCullMode::FrontFaceCulling;
		//triangle.materialIndex = matLambert_White;

		//m_Triangles.emplace_back(triangle);

		//Triangle Mesh
		//pMesh = AddTriangleMesh(TriangleCullMode::NoCulling, matLambert_White);
		//pMesh->positions = { { -0.75f, -1.0f, 0.0f}, { -0.75f, 1.0f, 0.0f}, { 0.75f, 1.0f, 1.0f }, { 0.75f, -1.0f, 0.0f} };
		//pMesh->indices = {
		//	0,1,2, //Triangle 1
		//	0,2,3 //Triangle 2

		//};

		//pMesh->CalculateNormals();

		//pMesh->Translate({ 0.0f, 1.5f, 0.0f });
		//pMesh->UpdateTransforms();

		//Mesh
		pMesh = AddTriangleMesh(TriangleCullMode::BackFaceCulling, matLambert_White);
		Utils::ParseOBJ("Resources/simple_cube.obj", pMesh->positions, pMesh->normals, pMesh->indices);

		pMesh->Scale({ 0.7f, 0.7f, 0.7f });
		pMesh->Translate({ 0.0f, 1.0f, 0.0f });

		pMesh->UpdateTransforms();

		//Light
		AddPointLight(Vector3{ 0.0f, 5.0f, 5.0f }, 50.f, ColorRGB{ 1.0f, 0.61f, 0.45f });//backLight
		AddPointLight(Vector3{ -2.5f, 5.0f, -5.0f }, 70.f, ColorRGB{ 1.0f, 0.8f, 0.45f });//front light lef
		AddPointLight(Vector3{ 2.5f, 2.5f, -5.0f }, 50.f, ColorRGB{ 0.34f, 0.47f, 0.68f });
	}

	void Scene_W4_TestScene::Update(Timer* pTimer)
	{
		Scene::Update(pTimer);

		pMesh->RotateY(PI_DIV_2 * pTimer->GetTotal());
		pMesh->UpdateTransforms();
	}

	void Scene_W4_ReferenceScene::Initialize()
	{
		sceneName = "Reference Scene";
		m_Camera.origin = { 0.f, 3.0f, -9.0f };
		m_Camera.SetFovAngle(45.0f);

		const auto matCT_GrayRoughMetal = AddMaterial(new Material_CookTorrence({ 0.972f, 0.960f, 0.915f }, 1.0f, 1.0f));
		const auto matCT_GrayMediumMetal = AddMaterial(new Material_CookTorrence({ 0.972f, 0.960f, 0.915f }, 1.0f, 0.6f));
		const auto matCT_GraySmoothMetal = AddMaterial(new Material_CookTorrence({ 0.972f, 0.960f, 0.915f }, 1.0f, 0.1f));
		const auto matCT_GrayRoughPlastic = AddMaterial(new Material_CookTorrence({ 0.75f, 0.75f, 0.75f }, 0.0f, 1.f));
		const auto matCT_GrayMediumPlastic = AddMaterial(new Material_CookTorrence({ 0.75f, 0.75f, 0.75f }, 0.0f, 0.6f));
		const auto matCT_GraySmoothPlastic = AddMaterial(new Material_CookTorrence({ 0.75f, 0.75f, 0.75f }, 0.0f, 0.1f));

		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert({ 0.49f, 0.57f, 0.57f }, 1.0f));
		const auto matLambert_White = AddMaterial(new Material_Lambert(colors::White, 1.f));

		//Plane
		AddPlane(Vector3{ 0.0f, 0.0f, 10.0f }, Vector3{ 0.0f, 0.0f, -1.0f }, matLambert_GrayBlue);; //Back
		AddPlane(Vector3{ 0.0f, 0.0f, 0.0f }, Vector3{ 0.0f, 1.0f, 0.0f }, matLambert_GrayBlue);; //Bottom
		AddPlane(Vector3{ 0.0f, 10.0f, 0.0f }, Vector3{ 0.0f, -1.0f, 0.0f }, matLambert_GrayBlue);; //Top
		AddPlane(Vector3{ 5.0f, 0.f, 0.0f }, Vector3{ -1.0f, 0.0f, 0.0f }, matLambert_GrayBlue);; //Right
		AddPlane(Vector3{ -5.0f, 0.0f, 0.0f }, Vector3{ 1.0f, 0.0f, 0.0f }, matLambert_GrayBlue);; //Left

		//Spheres
		AddSphere(Vector3{ -1.75f, 1.0f, 0.0f }, 0.75f, matCT_GrayRoughMetal);
		AddSphere(Vector3{ 0.0f, 1.0f, 0.0f }, 0.75f, matCT_GrayMediumMetal);
		AddSphere(Vector3{ 1.75f, 1.0f, 0.0f }, 0.75f, matCT_GraySmoothMetal);
		AddSphere(Vector3{ -1.75f, 3.0f, 0.0f }, 0.75f, matCT_GrayRoughPlastic);
		AddSphere(Vector3{ 0.0f, 3.0f, 0.0f }, 0.75f, matCT_GrayMediumPlastic);
		AddSphere(Vector3{ 1.75f, 3.0f, 0.0f }, 0.75f, matCT_GraySmoothPlastic);

		const Triangle baseTriangle = { Vector3(-0.75f, 1.5f, 0.0f), Vector3(0.75f, 0.0f, 0.0f), Vector3(-0.75f, 0.0f, 0.0f) };

		m_pMeshes[0] = AddTriangleMesh(TriangleCullMode::BackFaceCulling, matLambert_White);
		m_pMeshes[0]->AppendTriangle(baseTriangle, true);
#ifdef USE_BVH
		m_pMeshes[0]->pBvhNodes = new BVHNode[m_pMeshes[0]->indices.size()]{};
#endif
		m_pMeshes[0]->Translate({ -1.75f, 4.5f, 0.0f });
		m_pMeshes[0]->UpdateAABB();
		m_pMeshes[0]->UpdateTransforms();

		m_pMeshes[1] = AddTriangleMesh(TriangleCullMode::FrontFaceCulling, matLambert_White);
		m_pMeshes[1]->AppendTriangle(baseTriangle, true);
#ifdef USE_BVH
		m_pMeshes[1]->pBvhNodes = new BVHNode[m_pMeshes[1]->indices.size()]{};
#endif
		m_pMeshes[1]->Translate({ 0.0f, 4.5f, 0.0f });
		m_pMeshes[1]->UpdateAABB();
		m_pMeshes[1]->UpdateTransforms();

		m_pMeshes[2] = AddTriangleMesh(TriangleCullMode::NoCulling, matLambert_White);
		m_pMeshes[2]->AppendTriangle(baseTriangle, true);
#ifdef USE_BVH
		m_pMeshes[2]->pBvhNodes = new BVHNode[m_pMeshes[2]->indices.size()]{};
#endif
		m_pMeshes[2]->Translate({ 1.75f, 4.5f, 0.0f });
		m_pMeshes[2]->UpdateAABB();
		m_pMeshes[2]->UpdateTransforms();

		//Light
		AddPointLight(Vector3{ 0.0f, 5.0f, 5.0f }, 50.f, ColorRGB{ 1.0f, 0.61f, 0.45f }); // Backlight
		AddPointLight(Vector3{ -2.5f, 5.0f, -5.0f }, 70.f, ColorRGB{ 1.0f, 0.8f, 0.45f }); // Frontlight left
		AddPointLight(Vector3{ 2.5f, 2.5f, -5.0f }, 50.f, ColorRGB{ 0.34f, 0.47f, 0.68f });
	}

	void Scene_W4_ReferenceScene::Update(Timer* pTimer)
	{
		Scene::Update(pTimer);

		const float yawAngle{ (cosf(pTimer->GetTotal()) + 1.0f) / 2.0f * PI_2 };
		for (TriangleMesh* pMesh : m_pMeshes)
		{
			pMesh->RotateY(yawAngle);
			pMesh->UpdateTransforms();
		}
	}
	void Scene_W4_BunnyScene::Initialize()
	{
		sceneName = "Bunny Scene";
		m_Camera.origin = { 0.f, 3.0f, -9.0f };
		m_Camera.SetFovAngle(45.0f);

		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert({ 0.49f, 0.57f, 0.57f }, 1.0f));
		const auto matLambert_White = AddMaterial(new Material_Lambert(colors::White, 1.f));

		//Plane
		AddPlane(Vector3{ 0.0f, 0.0f, 10.0f }, Vector3{ 0.0f, 0.0f, -1.0f }, matLambert_GrayBlue);; //Back
		AddPlane(Vector3{ 0.0f, 0.0f, 0.0f }, Vector3{ 0.0f, 1.0f, 0.0f }, matLambert_GrayBlue);; //Bottom
		AddPlane(Vector3{ 0.0f, 10.0f, 0.0f }, Vector3{ 0.0f, -1.0f, 0.0f }, matLambert_GrayBlue);; //Top
		AddPlane(Vector3{ 5.0f, 0.f, 0.0f }, Vector3{ -1.0f, 0.0f, 0.0f }, matLambert_GrayBlue);; //Right
		AddPlane(Vector3{ -5.0f, 0.0f, 0.0f }, Vector3{ 1.0f, 0.0f, 0.0f }, matLambert_GrayBlue);; //Left

		//Mesh
		m_pMesh = AddTriangleMesh(TriangleCullMode::BackFaceCulling, matLambert_White);
		Utils::ParseOBJ("Resources/lowpoly_bunny2.obj", m_pMesh->positions, m_pMesh->normals, m_pMesh->indices);

		m_pMesh->Scale({ 2.0f, 2.0f, 2.0f });

#ifndef USE_BVH
		m_pMesh->UpdateAABB();
#endif
		m_pMesh->UpdateTransforms();

		//Light
		AddPointLight(Vector3{ 0.0f, 5.0f, 5.0f }, 50.f, ColorRGB{ 1.0f, 0.61f, 0.45f }); // Backlight
		AddPointLight(Vector3{ -2.5f, 5.0f, -5.0f }, 70.f, ColorRGB{ 1.0f, 0.8f, 0.45f }); // Frontlight left
		AddPointLight(Vector3{ 2.5f, 2.5f, -5.0f }, 50.f, ColorRGB{ 0.34f, 0.47f, 0.68f });
	}

	void Scene_W4_BunnyScene::Update(Timer* pTimer)
	{
		Scene::Update(pTimer);

		const float yawAngle{ (cosf(pTimer->GetTotal()) + 1.0f) / 2.0f * PI_2 };
		m_pMesh->RotateY(yawAngle);
		m_pMesh->UpdateTransforms();
	}
}

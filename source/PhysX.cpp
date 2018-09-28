//
//  PhysX.cpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#include "PhysX.hpp"

#include <iostream>

std::unique_ptr<PhysX> physicsEngine;

PhysX::PhysX() :
	m_defaultFilterShader(PxDefaultSimulationFilterShader)
{
	m_foundation = PxCreateFoundation(PX_FOUNDATION_VERSION, m_defaultAllocatorCallback, m_defaultErrorCallback);
	m_physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_foundation, PxTolerancesScale());
	if(m_physics == nullptr) {
		std::cerr << "Error creating PhysX3 device." << std::endl;
		std::cerr << "Exiting..." << std::endl;
		exit(1);
	}

	m_cpuDispatcher = PxDefaultCpuDispatcherCreate(1);
	if(!m_cpuDispatcher)
		std::cerr << "PxDefaultCpuDispatcherCreate failed!" << std::endl;
}

PhysX::~PhysX()
{
	m_cpuDispatcher->release();
	m_physics->release();
	m_foundation->release();
}

PxScene* PhysX::createPxScene() {
	//Create the scene
	PxSceneDesc sceneDesc(m_physics->getTolerancesScale());
	sceneDesc.gravity=PxVec3(0.0f, -9.8f, 0.0f);

	if(!sceneDesc.cpuDispatcher)
		sceneDesc.cpuDispatcher = m_cpuDispatcher;
	if(!sceneDesc.filterShader)
		sceneDesc.filterShader  = m_defaultFilterShader;


	PxScene* pScene = m_physics->createScene(sceneDesc);
	if (!pScene)
		std::cerr << "createScene failed!" << std::endl;

	return pScene;
}

static PxQuat QuatRotate(PxVec3 axis, PxReal rot) {
	PxQuat ret;
	axis /= axis.magnitude();
	ret.w = cos(rot / 2);
	ret.x = sin(rot / 2) * axis.x;
	ret.y = sin(rot / 2) * axis.y;
	ret.z = sin(rot / 2) * axis.z;
	return ret;
}
//static PxVec3 VecRotate (PxReal theta, PxReal phi) {
//	PxVec3 ret;
//	ret.x = PxSin(theta) * PxCos(phi);
//	ret.y = PxSin(theta) * PxSin(phi);
//	ret.z = PxCos(theta);
//	return ret;
//}

PxRigidDynamic* PhysX::CreateBox (PxVec3 pos, PxVec3 axis, PxReal rot, PxVec3 dimension,
						   PxReal density, PxMaterial* mMaterial) {
	PxTransform transform(pos, QuatRotate(axis, rot));
	PxBoxGeometry geometry(dimension);
	PxRigidDynamic *actor = PxCreateDynamic(*m_physics, transform, geometry, *mMaterial, density);
	if (!actor)
		std::cerr << "create actor failed!" << std::endl;
	actor->setAngularDamping(0.75);
	actor->setLinearVelocity(PxVec3(0, 0, 0));
	return actor;
}
PxRigidDynamic* PhysX::CreateSphere(PxVec3 pos, PxReal radius,
							 PxReal density, PxMaterial* mMaterial) {
	// Add a single-shape actor to the scene
	PxTransform transform(pos, PxQuat(PxIdentity));
	PxSphereGeometry geometry(radius);

	PxRigidDynamic *actor = PxCreateDynamic(*m_physics, transform, geometry, *mMaterial, density);
	if (!actor)
		std::cerr << "create actor failed!" << std::endl;
	actor->setAngularDamping(0.75);
	actor->setLinearVelocity(PxVec3(0,0,0));
	return actor;
}
PxRigidDynamic* PhysX::CreateCapsule(PxVec3 pos, PxVec3 axis, PxReal rot, PxReal radius, PxReal half_height,
							  PxReal density, PxMaterial *mMaterial) {
	PxTransform transform(pos, QuatRotate(axis, rot));
	PxCapsuleGeometry geometry(radius, half_height);
	PxRigidDynamic *actor = PxCreateDynamic(*m_physics, transform, geometry, *mMaterial, density);
	if (!actor)
		std::cerr << "create actor failed!" << std::endl;
	actor->setAngularDamping(0.75);
	actor->setLinearVelocity(PxVec3(0, 0, 0));
	return actor;
}

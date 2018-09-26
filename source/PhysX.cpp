//
//  PhysX.cpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#include "PhysX.hpp"

#include <iostream>

PhysX *physicsEngine;

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

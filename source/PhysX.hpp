//
//  PhysX.hpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#ifndef PhysX_hpp
#define PhysX_hpp

#include <PxPhysicsAPI.h>

using namespace physx;

class PhysX {
public:
	PhysX();
	~PhysX();
	
	PxScene* createPxScene();
	PxPhysics* getPhysics() { return m_physics; };
	
private:
	PxFoundation			 *m_foundation    = nullptr;
	PxPhysics				 *m_physics    	  = nullptr;
	PxDefaultCpuDispatcher	 *m_cpuDispatcher = nullptr;
	PxDefaultErrorCallback	 m_defaultErrorCallback;
	PxDefaultAllocator		 m_defaultAllocatorCallback;
	PxSimulationFilterShader m_defaultFilterShader;
};

extern PhysX *physicsEngine;

#endif /* PhysX_hpp */

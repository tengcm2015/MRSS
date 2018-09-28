//
//  PhysX.hpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#ifndef PhysX_hpp
#define PhysX_hpp

#include <PxPhysicsAPI.h>

#include <memory>

using namespace physx;

class PhysX {
public:
	PhysX();
	~PhysX();

	PxScene* createPxScene();
	PxPhysics* getPhysics() { return m_physics; };

	PxRigidDynamic* CreateBox (PxVec3 pos, PxVec3 axis, PxReal rot,
							   PxVec3 dimension, PxReal density, PxMaterial* mMaterial);
	PxRigidDynamic* CreateSphere(PxVec3 pos, PxReal radius,
								 PxReal density, PxMaterial* mMaterial);
	PxRigidDynamic* CreateCapsule(PxVec3 pos, PxVec3 axis, PxReal rot,
								  PxReal radius, PxReal half_height,
								  PxReal density, PxMaterial *mMaterial);

private:
	PxFoundation			 *m_foundation    = nullptr;
	PxPhysics				 *m_physics    	  = nullptr;
	PxDefaultCpuDispatcher	 *m_cpuDispatcher = nullptr;
	PxDefaultErrorCallback	 m_defaultErrorCallback;
	PxDefaultAllocator		 m_defaultAllocatorCallback;
	PxSimulationFilterShader m_defaultFilterShader;
};

extern std::unique_ptr<PhysX> physicsEngine;

#endif /* PhysX_hpp */

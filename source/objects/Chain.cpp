//
//  Chain.cpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#include "Chain.hpp"

//Chain constructor
Chain::Chain(PxVec3 pos, int seg_num, PxReal seg_radius, PxReal seg_half_length,
			 PxReal density, PxMaterial *mMaterial) {
	// top segment = seg 0  // seg_num > 0
	
	seg_joint_dist = 1.10 * seg_radius + seg_half_length;
	PxCapsuleGeometry geometry(seg_radius, seg_half_length);
	
	PxTransform position(pos, PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	PxTransform downend(PxVec3(-seg_joint_dist, 0, 0));
	PxTransform upend(PxVec3(seg_joint_dist, 0, 0));
	
	articulation = gPhysicsSDK->createArticulation();
	PxArticulationLink *u_link = articulation->createLink(NULL, position);
	u_link->createShape(geometry, *mMaterial);
	PxRigidBodyExt::updateMassAndInertia(*u_link, density);
	ArticLinks.push_back(u_link);
	
	//	global position this time
	for (int i = 1; i < seg_num; i++) {
		position.p -= PxVec3(0, 2 * seg_joint_dist, 0);
		PxArticulationLink *d_link = articulation->createLink(u_link, position);
		d_link->createShape(geometry, *mMaterial);
		PxRigidBodyExt::updateMassAndInertia(*d_link, density);
		ArticLinks.push_back(d_link);
		
		PxArticulationJoint *j = d_link->getInboundJoint();
		j->setParentPose(downend);
		j->setChildPose(upend);
		j->setTwistLimit(0, 0);
		j->setTwistLimitEnabled(true);
		
		u_link = d_link;
	}
	
	gScene->addArticulation(*articulation);
	
}

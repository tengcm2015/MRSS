//
//  Swing.cpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#include "Swing.hpp"

#include <iostream>

//Swing constructor
Swing::Swing(PxVec3 pos, PxReal half_width, PxReal seg_num, PxReal seg_radius, PxReal seg_half_length,
			 PxReal chain_density, PxMaterial *chain_material,
			 PxReal seat_density, PxMaterial *seat_material) {
	
	seg_joint_dist = 1.10 * seg_radius + seg_half_length;
	str_head_height = pos.y - 0.21 - seg_joint_dist;
	seat_height = pos.y - 0.21 - (2 * (seg_joint_dist) * (seg_num + 1))
	+ (1.10 * seg_radius);
	
	//shaft
	PxTransform shaft_pos(pos);
	
	PxCapsuleGeometry shaft_geom(0.2, 5.0);
	//    shaft = PxCreateStatic(*gPhysicsSDK, shaft_pos, shaft_geom, *chain_material);
	shaft = gPhysicsSDK->createRigidStatic(shaft_pos);
	shaft_stick = shaft->createShape(shaft_geom, *chain_material);
	if (!shaft)
		std::cerr << "create actor failed!" << std::endl;
	gScene->addActor(*shaft);
	
	// chains
	PxVec3 chain_head1(half_width, str_head_height, 0);
	PxVec3 chain_head2(-half_width, str_head_height, 0);
	
	chain1 = new Chain(chain_head1, seg_num, seg_radius, seg_half_length,
					   chain_density, chain_material);
	chain2 = new Chain(chain_head2, seg_num, seg_radius, seg_half_length,
					   chain_density, chain_material);
	
	//seat
	PxVec3 seat_dim(half_width, 0.1, half_width / 2);
	PxTransform seat_pos(PxVec3(0, seat_height, 0));
	PxTransform connector1 = PxTransform(PxVec3(half_width, seg_half_length, 0),
										 PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	PxTransform connector2 = PxTransform(PxVec3(-half_width, seg_half_length, 0),
										 PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	
	seat = gPhysicsSDK->createRigidDynamic(seat_pos);
	if (!seat)
		std::cerr << "create actor failed!" << std::endl;
	
	seat_board = (seat->createShape(PxBoxGeometry(seat_dim), *seat_material));
	
	PxShape *shape;
	
	shape = gPhysicsSDK->createShape(PxCapsuleGeometry(seg_radius, seg_half_length), *chain_material);
	shape->setLocalPose(connector1);
	seat->attachShape(*shape);
	
	shape = gPhysicsSDK->createShape(PxCapsuleGeometry(seg_radius, seg_half_length), *chain_material);
	shape->setLocalPose(connector2);
	seat->attachShape(*shape);
	
	PxRigidBodyExt::updateMassAndInertia(*seat, seat_density);
	seat->setAngularDamping(0.75);
	seat->setLinearVelocity(PxVec3(0,0,0));
	gScene->addActor(*seat);
	
	// put things together
	PxTransform shaft_joint1(PxVec3(half_width, 0, 0), PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	PxTransform shaft_joint2(PxVec3(-half_width, 0, 0), PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	
	PxTransform downend(PxVec3(-seg_joint_dist, 0, 0));
	PxTransform upend(PxVec3(seg_joint_dist + 0.3, 0, 0));
	
	PxD6Joint *j;
	
	j = PxD6JointCreate(*gPhysicsSDK, shaft, shaft_joint1, chain1->ArticLinks[0], upend);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE); //free to rotate around y axis
	
	j = PxD6JointCreate(*gPhysicsSDK, shaft, shaft_joint2, chain2->ArticLinks[0], upend);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE); //free to rotate around y axis
	
	connector1.p += PxVec3(0, seg_joint_dist, 0);
	connector2.p += PxVec3(0, seg_joint_dist, 0);
	
	j = PxD6JointCreate(*gPhysicsSDK, chain1->ArticLinks[seg_num - 1], downend, seat, connector1);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE); //free to rotate around y axis
	
	j = PxD6JointCreate(*gPhysicsSDK, chain2->ArticLinks[seg_num - 1], downend, seat, connector2);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE); //free to rotate around y axis
	
}


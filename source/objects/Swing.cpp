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
			 PxReal seat_density, PxMaterial *seat_material) :
	BaseObject()
{
	PxPhysics *gPhysicsSDK = physicsEngine->getPhysics();

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
	m_shaft = std::make_unique<SingleObject>(shaft);

	// chains
	PxVec3 chain_head1(half_width, str_head_height, 0);
	PxVec3 chain_head2(-half_width, str_head_height, 0);

	m_chains[0] = std::make_unique<Chain>(chain_head1, seg_num, seg_radius, seg_half_length, chain_density, chain_material);
	m_chains[1] = std::make_unique<Chain>(chain_head2, seg_num, seg_radius, seg_half_length, chain_density, chain_material);

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
	m_seat = std::make_unique<SingleObject>(seat);

	// put things together
	PxTransform shaft_joint1(PxVec3(half_width, 0, 0), PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
	PxTransform shaft_joint2(PxVec3(-half_width, 0, 0), PxQuat(PxHalfPi, PxVec3(0, 0, 1)));

	PxTransform downend(PxVec3(-seg_joint_dist, 0, 0));
	PxTransform upend(PxVec3(seg_joint_dist + 0.3, 0, 0));

	PxD6Joint *j;

	j = PxD6JointCreate(*gPhysicsSDK, shaft, shaft_joint1, m_chains[0]->getLink(0)->getActor(), upend);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE); //free to rotate around y axis

	j = PxD6JointCreate(*gPhysicsSDK, shaft, shaft_joint2, m_chains[1]->getLink(0)->getActor(), upend);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE); //free to rotate around y axis

	connector1.p += PxVec3(0, seg_joint_dist, 0);
	connector2.p += PxVec3(0, seg_joint_dist, 0);

	j = PxD6JointCreate(*gPhysicsSDK, m_chains[0]->getLink(seg_num - 1)->getActor(), downend, seat, connector1);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE); //free to rotate around y axis

	j = PxD6JointCreate(*gPhysicsSDK, m_chains[1]->getLink(seg_num - 1)->getActor(), downend, seat, connector2);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE); //free to rotate around y axis

}

Swing::~Swing()
{
}

void Swing::addToPxScene(PxScene *scene) {
	m_shaft->addToPxScene(scene);
	m_seat->addToPxScene(scene);
	m_chains[0]->addToPxScene(scene);
	m_chains[1]->addToPxScene(scene);
}

void Swing::removeFromPxScene(PxScene *scene) {
	m_shaft->removeFromPxScene(scene);
	m_seat->removeFromPxScene(scene);
	m_chains[0]->removeFromPxScene(scene);
	m_chains[1]->removeFromPxScene(scene);
}

void Swing::draw() {
	m_shaft->draw();
	m_seat->draw();
	m_chains[0]->draw();
	m_chains[1]->draw();
}

// return angle by degree
float Swing::getAngle() {
	PxVec3 shaft_pos = (PxShapeExt::getGlobalPose(*shaft_stick, *shaft)).p;
	PxVec3 seat_pos = (PxShapeExt::getGlobalPose(*seat_board, *seat)).p;
	PxVec3 p = seat_pos - shaft_pos;
	return atanf(-p.z/p.y) / 3.14 * 180;
}

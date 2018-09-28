//
//  SwingTester.cpp
//  MRSS
//
//  Created by tengcm on 2018/9/28.
//

#include "SwingTester.hpp"

#include <iostream>

static const MannequinRot mode_crouch = {
	PxPi * -1 / 4,
	PxPi * 2 / 3,
	PxPi * -1 / 2,
	PxPi * 1 / 2
};
static const MannequinRot mode_stand = {
	PxPi * 3 / 4,
	PxPi * -1 / 2,
	PxPi * 0 / 3,
	PxPi * -1 / 6
};


SwingTester::SwingTester(double groundX, double groundY) :
	BaseObject(),
	m_max_angle(0),
	m_min_angle(0),
	m_pose(TESTER_POSE::STAND)
{
	PxPhysics *gPhysicsSDK = physicsEngine->getPhysics();

	// Create Swing
	double swing_height = 10.0;
	double seg_radius = 0.07;
	double seg_half_length = 0.30;
	int seg_num = 10;
	double half_width = 1.0;

	double mannequin_height = 6;
	double shoulder_width = half_width * 1.5;

	PxMaterial *body = gPhysicsSDK->createMaterial(0.7, 0.65, 0.1);
	PxMaterial *iron = gPhysicsSDK->createMaterial(0.5, 0.4, 0.65);
	PxMaterial *wood = gPhysicsSDK->createMaterial(0.5, 0.4, 0.5);

	PxReal body_density = 2.0f;
	PxReal iron_density = 7.8f;
	PxReal wood_density = 5.4f;

	m_swing = std::make_unique<Swing>(PxVec3(groundX, swing_height, groundY), half_width, seg_num, seg_radius, seg_half_length,
						 iron_density, iron, wood_density, wood);

	m_mannequin = std::make_unique<Mannequin>(mannequin_height, shoulder_width,
								 PxVec3(groundX, m_swing->seat_height + 0.2, groundY), body_density, body);

	// Make Mannequin grasp the chain
	int hand_seg_num = 5;
	PxTransform arm_hand(PxVec3(-(m_mannequin->arm_length / 2), 0, 0));
	PxD6Joint *j;

	j = PxD6JointCreate(*gPhysicsSDK, m_mannequin->lower_arm1, arm_hand, m_swing->getChain(0)->getLink(hand_seg_num)->getActor(), PxTransform(PxVec3(0)));
	j->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);

	j = PxD6JointCreate(*gPhysicsSDK, m_mannequin->lower_arm2, arm_hand, m_swing->getChain(1)->getLink(hand_seg_num)->getActor(), PxTransform(PxVec3(0)));
	j->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);

	PxTransform leg_foot(PxVec3(-(m_mannequin->leg_length / 2 + 0.2), 0, 0));
	PxVec3 seat_foot((m_mannequin->body_width - m_mannequin->leg_width - 0.1) / 2, 0, 0);

	j = PxD6JointCreate(*gPhysicsSDK, m_mannequin->lower_leg1, leg_foot,
						m_swing->seat, PxTransform(seat_foot));
	j->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

	j = PxD6JointCreate(*gPhysicsSDK, m_mannequin->lower_leg2, leg_foot,
						m_swing->seat, PxTransform(-seat_foot));
	j->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

}

SwingTester::~SwingTester()
{
}

void SwingTester::addToPxScene(PxScene *scene) {
	m_swing->addToPxScene(scene);
	m_mannequin->addToPxScene(scene);
}

void SwingTester::removeFromPxScene(PxScene *scene) {
	m_swing->removeFromPxScene(scene);
	m_mannequin->removeFromPxScene(scene);
}

void SwingTester::draw() {
	m_swing->draw();
	m_mannequin->draw();
}

void SwingTester::setDriveOn(bool driveOn) {
	if (driveOn == true)
		m_mannequin->driveOn();
	else
		m_mannequin->driveOff();
}

void SwingTester::setPose(TESTER_POSE pose) {
	m_pose = pose;
	if (m_pose == TESTER_POSE::STAND)
		m_mannequin->setMode(mode_crouch);
	else
		m_mannequin->setMode(mode_stand);
}

void SwingTester::autoPose() {
	float currentAngle = m_swing->getAngle();
	if (!m_mannequin->isLocked()) {
		if (currentAngle > m_max_angle) {
			m_max_angle = currentAngle;
		}
		if (currentAngle < m_min_angle) {
			m_min_angle = currentAngle;
		}
		float range = (m_max_angle - m_min_angle) / 6;
		float offset = 0;

		if (currentAngle > range+offset || currentAngle < -range+offset) {
			this->setPose(TESTER_POSE::CROUCH);
		} else {
			this->setPose(TESTER_POSE::STAND);
		}
	}
}

void SwingTester::resetAutoState() {
	m_max_angle = m_min_angle = 0;
}

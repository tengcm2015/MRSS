//
//  Mannequin.cpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#include "Mannequin.hpp"

static PxQuat QuatRotate(PxVec3 axis, PxReal rot) {
	PxQuat ret;
	axis /= axis.magnitude();
	ret.w = cos(rot / 2);
	ret.x = sin(rot / 2) * axis.x;
	ret.y = sin(rot / 2) * axis.y;
	ret.z = sin(rot / 2) * axis.z;
	return ret;
}

Mannequin::Mannequin(double height, double shoulder_width, PxVec3 foot_position, PxReal density, PxMaterial *mMaterial) :
	BaseObject(),
	m_locked(true),
	m_drive_on(false),
	m_D6_drive(std::make_unique<PxD6JointDrive>(0.0f, 0.0f, PX_MAX_F32, true)),
	m_rot({
		PxPi * -1 / 4,
		PxPi * 2 / 3,
		PxPi * -1 / 2,
		PxPi * 1 / 2
	})
{
	PxPhysics *gPhysicsSDK = physicsEngine->getPhysics();

	//height: capsule w/o sphere part
	//length: capsule w/ sphere part
	arm_width	= shoulder_width / 5;
	arm_height	= arm_width * 2.5;
	arm_length	= arm_height + arm_width;

	leg_width	= arm_width * 1.125;

	body_width	= 3 * arm_width;
	leg_height	= (height - (body_width * 2 + leg_width * 2)) / 3;
	leg_length	= leg_width + leg_height;

	body_height	= leg_height;
	body_length	= body_height + body_width;

	head_radius	= body_width / 2;

	//global pose
	PxVec3 body_pos = foot_position + PxVec3(0, (leg_length * 2) + (body_length / 2), 0);

	PxVec3 body_to_shoulder1 ((body_width + arm_width) / 2, body_height / 2, 0);
	PxVec3 body_to_shoulder2 (-(body_width + arm_width) / 2, body_height / 2, 0);
	PxVec3 upper_to_lower_arm (0, -arm_length, 0);
	PxVec3 shoulder_to_upper_arm = upper_to_lower_arm / 2;

	PxVec3 body_to_leg_root1 ((body_width - leg_width - 0.1) / 2, -body_length / 2, 0);
	PxVec3 body_to_leg_root2 (-(body_width - leg_width - 0.1) / 2, -body_length / 2, 0);

	PxVec3 upper_to_lower_leg (0, -leg_length, 0);
	PxVec3 root_to_upper_leg = upper_to_lower_leg / 2;

	head_act = physicsEngine->CreateSphere(body_pos + PxVec3(0, body_length / 2 + head_radius, 0), head_radius, density, mMaterial);
	m_body_parts.insert(std::make_pair(MANNEQUIN::HEAD, SingleObject(head_act)));

	body_act = physicsEngine->CreateCapsule(body_pos, PxVec3(0, 0, 1), PxHalfPi, body_width / 2, body_height / 2, density, mMaterial);
	m_body_parts.insert(std::make_pair(MANNEQUIN::BODY, SingleObject(body_act)));

	upper_arm1 = physicsEngine->CreateCapsule(body_pos + body_to_shoulder1 + shoulder_to_upper_arm, PxVec3(0, 0, 1), PxHalfPi, arm_width / 2, arm_height / 2, density, mMaterial);
	m_body_parts.insert(std::make_pair(MANNEQUIN::U_ARM_L, SingleObject(upper_arm1)));

	upper_arm2 = physicsEngine->CreateCapsule(body_pos + body_to_shoulder2 + shoulder_to_upper_arm, PxVec3(0, 0, 1), PxHalfPi, arm_width / 2, arm_height / 2, density, mMaterial);
	m_body_parts.insert(std::make_pair(MANNEQUIN::U_ARM_R, SingleObject(upper_arm2)));

	lower_arm1 = physicsEngine->CreateCapsule(body_pos + body_to_shoulder1 + shoulder_to_upper_arm + upper_to_lower_arm, PxVec3(0, 0, 1), PxHalfPi, arm_width / 2, arm_height / 2, density, mMaterial);
	m_body_parts.insert(std::make_pair(MANNEQUIN::L_ARM_L, SingleObject(lower_arm1)));

	lower_arm2 = physicsEngine->CreateCapsule(body_pos + body_to_shoulder2 + shoulder_to_upper_arm + upper_to_lower_arm, PxVec3(0, 0, 1), PxHalfPi, arm_width / 2, arm_height / 2, density, mMaterial);
	m_body_parts.insert(std::make_pair(MANNEQUIN::L_ARM_R, SingleObject(lower_arm2)));

	upper_leg1 = physicsEngine->CreateCapsule(body_pos + body_to_leg_root1 + root_to_upper_leg, PxVec3(0, 0, 1), PxHalfPi, leg_width / 2, leg_height / 2, density, mMaterial);
	m_body_parts.insert(std::make_pair(MANNEQUIN::U_LEG_L, SingleObject(upper_leg1)));

	upper_leg2 = physicsEngine->CreateCapsule(body_pos + body_to_leg_root2 + root_to_upper_leg, PxVec3(0, 0, 1), PxHalfPi, leg_width / 2, leg_height / 2, density, mMaterial);
	m_body_parts.insert(std::make_pair(MANNEQUIN::U_LEG_R, SingleObject(upper_leg2)));

	lower_leg1 = physicsEngine->CreateCapsule(body_pos + body_to_leg_root1 + root_to_upper_leg + upper_to_lower_leg, PxVec3(0, 0, 1), PxHalfPi, leg_width / 2, leg_height / 2, density, mMaterial);
	m_body_parts.insert(std::make_pair(MANNEQUIN::L_LEG_L, SingleObject(lower_leg1)));

	lower_leg2 = physicsEngine->CreateCapsule(body_pos + body_to_leg_root2 + root_to_upper_leg + upper_to_lower_leg, PxVec3(0, 0, 1), PxHalfPi, leg_width / 2, leg_height / 2, density, mMaterial);
	m_body_parts.insert(std::make_pair(MANNEQUIN::L_LEG_R, SingleObject(lower_leg2)));

	//local pose here
	//caution: NOT y!
	PxTransform body_upend		(PxVec3(body_length / 2, 0, 0));
	//this one is different
	PxTransform head_downend	(PxVec3(0, -head_radius, 0), QuatRotate(PxVec3(0, 0, 1), PxHalfPi));

	PxTransform body_to_sho1	(PxVec3(body_to_shoulder1.y, -body_to_shoulder1.x, body_to_shoulder1.z));
	PxTransform body_to_sho2	(PxVec3(body_to_shoulder2.y, -body_to_shoulder2.x, body_to_shoulder2.z));
	PxTransform body_to_root1	(PxVec3(body_to_leg_root1.y, -body_to_leg_root1.x, body_to_leg_root1.z));
	PxTransform body_to_root2	(PxVec3(body_to_leg_root2.y, -body_to_leg_root2.x, body_to_leg_root2.z));

	PxVec3 arm_end(arm_length / 2, 0, 0);
	PxVec3 leg_end(leg_length / 2, 0, 0);

	PxQuat arm_upend1	= QuatRotate(PxVec3(0, 1, 0), -PxHalfPi / 2);
	PxQuat arm_upend2	= QuatRotate(PxVec3(0, 1, 0), -PxHalfPi / 2);
	PxQuat arm_downend	= QuatRotate(PxVec3(0, 1, 0), PxHalfPi);
	PxQuat leg_upend1	= QuatRotate(PxVec3(0, 1, 0), -PxHalfPi);
	PxQuat leg_upend2	= QuatRotate(PxVec3(0, 1, 0), -PxHalfPi);
	PxQuat leg_downend	= QuatRotate(PxVec3(0, 1, 0), -PxHalfPi);

	// hold the body in place while attaching joint to the swing
	body_act->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

	j_neck = PxD6JointCreate(*gPhysicsSDK, head_act, head_downend, body_act, body_upend);
	MannequinJoints.push_back(j_neck);

	j_sho_1 = PxD6JointCreate(*gPhysicsSDK, body_act, body_to_sho1,
							  upper_arm1, PxTransform(arm_end, arm_upend1));
	j_sho_1->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j_sho_1->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED); //rotate around y axis
	j_sho_1->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED); //rotate around z axis
	j_sho_1->setSwingLimit(PxJointLimitCone(PxHalfPi * 4 / 3, PxHalfPi));
	MannequinJoints.push_back(j_sho_1);

	j_sho_2 = PxD6JointCreate(*gPhysicsSDK, body_act, body_to_sho2,
							  upper_arm2, PxTransform(arm_end, arm_upend2));
	j_sho_2->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j_sho_2->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
	j_sho_2->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
	j_sho_2->setSwingLimit(PxJointLimitCone(PxHalfPi * 4 / 3, PxHalfPi));
	MannequinJoints.push_back(j_sho_2);

	j_arm_1 = PxD6JointCreate(*gPhysicsSDK, upper_arm1, PxTransform(-arm_end, arm_downend),
							  lower_arm1, PxTransform(arm_end));
	j_arm_1->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
	j_arm_1->setSwingLimit(PxJointLimitCone(PxHalfPi * 3 / 4, 0));
	MannequinJoints.push_back(j_arm_1);

	j_arm_2 = PxD6JointCreate(*gPhysicsSDK, upper_arm2, PxTransform(-arm_end, arm_downend),
							  lower_arm2, PxTransform(arm_end));
	j_arm_2->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
	j_arm_2->setSwingLimit(PxJointLimitCone(PxHalfPi * 3 / 4, 0));
	MannequinJoints.push_back(j_arm_2);

	j_leg_1 = PxD6JointCreate(*gPhysicsSDK, body_act, body_to_root1,
							  upper_leg1, PxTransform(leg_end, leg_upend1));
	j_leg_1->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j_leg_1->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
	j_leg_1->setSwingLimit(PxJointLimitCone(PxHalfPi, PxHalfPi));
	MannequinJoints.push_back(j_leg_1);

	j_leg_2 = PxD6JointCreate(*gPhysicsSDK, body_act, body_to_root2,
							  upper_leg2, PxTransform(leg_end, leg_upend2));
	j_leg_2->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j_leg_2->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
	j_leg_2->setSwingLimit(PxJointLimitCone(PxHalfPi, PxHalfPi));
	MannequinJoints.push_back(j_leg_2);

	j_kne_1 = PxD6JointCreate(*gPhysicsSDK, upper_leg1, PxTransform(-leg_end, leg_downend),
							  lower_leg1, PxTransform(leg_end));
	j_kne_1->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
	j_kne_1->setSwingLimit(PxJointLimitCone(PxHalfPi, 0));
	MannequinJoints.push_back(j_kne_1);

	j_kne_2 = PxD6JointCreate(*gPhysicsSDK, upper_leg2, PxTransform(-leg_end, leg_downend),
							  lower_leg2, PxTransform(leg_end));
	j_kne_2->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
	j_kne_2->setSwingLimit(PxJointLimitCone(PxHalfPi, 0));
	MannequinJoints.push_back(j_kne_2);

}

Mannequin::~Mannequin()
{
}

//	force = spring * (targetPosition - position) + damping * (targetVelocity - velocity)
void Mannequin::driveOn() {
	m_drive_on = true;
	m_D6_drive->damping = 250.0f;
	m_D6_drive->stiffness = 4000.0f;
	j_sho_1->setDrive(PxD6Drive::eSWING, *m_D6_drive);
	j_sho_2->setDrive(PxD6Drive::eSWING, *m_D6_drive);
	j_arm_1->setDrive(PxD6Drive::eSWING, *m_D6_drive);
	j_arm_2->setDrive(PxD6Drive::eSWING, *m_D6_drive);

	m_D6_drive->stiffness = 3000.0f;
	j_leg_1->setDrive(PxD6Drive::eSWING, *m_D6_drive);
	j_leg_2->setDrive(PxD6Drive::eSWING, *m_D6_drive);
	j_kne_1->setDrive(PxD6Drive::eSWING, *m_D6_drive);
	j_kne_2->setDrive(PxD6Drive::eSWING, *m_D6_drive);
}

void Mannequin::driveOff() {
	m_drive_on = false;
	m_D6_drive->damping = 0.0f;
	m_D6_drive->stiffness = 0.0f;
	for (auto j = MannequinJoints.begin(); j != MannequinJoints.end(); j++)
	for (int i = 1; i < MannequinJoints.size(); i++) {
		MannequinJoints[i]->setDrive(PxD6Drive::eSWING, *m_D6_drive);
		MannequinJoints[i]->setDriveVelocity(PxVec3(PxZero), PxVec3(PxZero));
	}

}

void Mannequin::setMode(MannequinRot m) {
	m_rot = m;

	//local pose
	j_sho_1->setDrivePosition(PxTransform(PxQuat(m_rot.armrot_u, PxVec3(0, 1, 0))));
	j_sho_2->setDrivePosition(PxTransform(PxQuat(m_rot.armrot_u, PxVec3(0, 1, 0))));
	j_arm_1->setDrivePosition(PxTransform(PxQuat(m_rot.armrot_l, PxVec3(0, 1, 0))));
	j_arm_2->setDrivePosition(PxTransform(PxQuat(m_rot.armrot_l, PxVec3(0, 1, 0))));
	j_leg_1->setDrivePosition(PxTransform(PxQuat(m_rot.legrot_u, PxVec3(0, 1, 0))));
	j_leg_2->setDrivePosition(PxTransform(PxQuat(m_rot.legrot_u, PxVec3(0, 1, 0))));
	j_kne_1->setDrivePosition(PxTransform(PxQuat(m_rot.legrot_l, PxVec3(0, 1, 0))));
	j_kne_2->setDrivePosition(PxTransform(PxQuat(m_rot.legrot_l, PxVec3(0, 1, 0))));
}

void Mannequin::unlock() {
	m_locked = false;
	body_act->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
}

void Mannequin::addToPxScene(PxScene *scene) {
	for (auto o = m_body_parts.begin(); o != m_body_parts.end(); o++)
		o->second.addToPxScene(scene);
}

void Mannequin::removeFromPxScene(PxScene *scene) {
	for (auto o = m_body_parts.begin(); o != m_body_parts.end(); o++)
		o->second.removeFromPxScene(scene);
}

void Mannequin::draw() {
	for (auto o = m_body_parts.begin(); o != m_body_parts.end(); o++)
		o->second.draw();
}

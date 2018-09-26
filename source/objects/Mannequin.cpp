//
//  Mannequin.cpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#include "Mannequin.hpp"

Mannequin::Mannequin(double height, double shoulder_width, PxVec3 foot_position,
					 PxReal density, PxMaterial *mMaterial) :
	mode({
		1,
		PxPi * -1 / 4,
		PxPi * 2 / 3,
		PxPi * -1 / 2,
		PxPi * 1 / 2
	})
{
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
	
	//create drive
	drive = new PxD6JointDrive(0.0f, 0.0f, PX_MAX_F32, true);
	
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
	
	head_act = CreateSphere(body_pos + PxVec3(0, body_length / 2 + head_radius, 0), head_radius, density, mMaterial);
	
	body_act = CreateCapsule(body_pos, PxVec3(0, 0, 1), PxHalfPi, body_width / 2, body_height / 2,
							 density, mMaterial);
	
	upper_arm1 = CreateCapsule(body_pos + body_to_shoulder1 + shoulder_to_upper_arm,
							   PxVec3(0, 0, 1), PxHalfPi, arm_width / 2, arm_height / 2,
							   density, mMaterial);
	
	upper_arm2 = CreateCapsule(body_pos + body_to_shoulder2 + shoulder_to_upper_arm,
							   PxVec3(0, 0, 1), PxHalfPi, arm_width / 2, arm_height / 2,
							   density, mMaterial);
	lower_arm1 = CreateCapsule(body_pos + body_to_shoulder1 + shoulder_to_upper_arm + upper_to_lower_arm,
							   PxVec3(0, 0, 1), PxHalfPi, arm_width / 2, arm_height / 2,
							   density, mMaterial);
	
	lower_arm2 = CreateCapsule(body_pos + body_to_shoulder2 + shoulder_to_upper_arm + upper_to_lower_arm,
							   PxVec3(0, 0, 1), PxHalfPi, arm_width / 2, arm_height / 2,
							   density, mMaterial);
	
	upper_leg1 = CreateCapsule(body_pos + body_to_leg_root1 + root_to_upper_leg,
							   PxVec3(0, 0, 1), PxHalfPi, leg_width / 2, leg_height / 2,
							   density, mMaterial);
	
	upper_leg2 = CreateCapsule(body_pos + body_to_leg_root2 + root_to_upper_leg,
							   PxVec3(0, 0, 1), PxHalfPi, leg_width / 2, leg_height / 2,
							   density, mMaterial);
	
	lower_leg1 = CreateCapsule(body_pos + body_to_leg_root1 + root_to_upper_leg + upper_to_lower_leg,
							   PxVec3(0, 0, 1), PxHalfPi, leg_width / 2, leg_height / 2,
							   density, mMaterial);
	
	lower_leg2 = CreateCapsule(body_pos + body_to_leg_root2 + root_to_upper_leg + upper_to_lower_leg,
							   PxVec3(0, 0, 1), PxHalfPi, leg_width / 2, leg_height / 2,
							   density, mMaterial);
	
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
	globalState.locked = 1;
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
	j_arm_1->setSwingLimit(PxJointLimitCone(PxHalfPi, 0));
	MannequinJoints.push_back(j_arm_1);
	
	j_arm_2 = PxD6JointCreate(*gPhysicsSDK, upper_arm2, PxTransform(-arm_end, arm_downend),
							  lower_arm2, PxTransform(arm_end));
	j_arm_2->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
	j_arm_2->setSwingLimit(PxJointLimitCone(PxHalfPi, 0));
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

//	force = spring * (targetPosition - position) + damping * (targetVelocity - velocity)
void Mannequin::DriveOn() {
	active = 1;
	drive->damping = 250.0f;
	drive->stiffness = 4000.0f;
	j_sho_1->setDrive(PxD6Drive::eSWING, *drive);
	j_sho_2->setDrive(PxD6Drive::eSWING, *drive);
	j_arm_1->setDrive(PxD6Drive::eSWING, *drive);
	j_arm_2->setDrive(PxD6Drive::eSWING, *drive);
	
	drive->stiffness = 3000.0f;
	j_leg_1->setDrive(PxD6Drive::eSWING, *drive);
	j_leg_2->setDrive(PxD6Drive::eSWING, *drive);
	j_kne_1->setDrive(PxD6Drive::eSWING, *drive);
	j_kne_2->setDrive(PxD6Drive::eSWING, *drive);
	
	//local pose
	j_sho_1->setDrivePosition(PxTransform(PxQuat(mode.armrot_u, PxVec3(0, 1, 0))));
	j_sho_2->setDrivePosition(PxTransform(PxQuat(mode.armrot_u, PxVec3(0, 1, 0))));
	j_arm_1->setDrivePosition(PxTransform(PxQuat(mode.armrot_l, PxVec3(0, 1, 0))));
	j_arm_2->setDrivePosition(PxTransform(PxQuat(mode.armrot_l, PxVec3(0, 1, 0))));
	j_leg_1->setDrivePosition(PxTransform(PxQuat(mode.legrot_u, PxVec3(0, 1, 0))));
	j_leg_2->setDrivePosition(PxTransform(PxQuat(mode.legrot_u, PxVec3(0, 1, 0))));
	j_kne_1->setDrivePosition(PxTransform(PxQuat(mode.legrot_l, PxVec3(0, 1, 0))));
	j_kne_2->setDrivePosition(PxTransform(PxQuat(mode.legrot_l, PxVec3(0, 1, 0))));
	
}
void Mannequin::DriveOff() {
	active = 0;
	drive->damping = 0.0f;
	drive->stiffness = 0.0f;
	for (int i = 1; i < MannequinJoints.size(); i++) {
		MannequinJoints[i]->setDrive(PxD6Drive::eSWING, *drive);
		MannequinJoints[i]->setDriveVelocity(PxVec3(PxZero), PxVec3(PxZero));
	}
	
}

void Mannequin::setMode(MannequinRot m) {
	mode = m;
	DriveOn();
}

void Mannequin::Unlock() {
	globalState.locked = 0;
	body_act->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
}
void Mannequin::Clear() {
	gScene->removeActor(*head_act);
	gScene->removeActor(*body_act);
	gScene->removeActor(*upper_arm1);
	gScene->removeActor(*upper_arm2);
	gScene->removeActor(*lower_arm1);
	gScene->removeActor(*lower_arm2);
	gScene->removeActor(*upper_leg1);
	gScene->removeActor(*upper_leg2);
	gScene->removeActor(*lower_leg1);
	gScene->removeActor(*lower_leg2);
	delete drive;
}
void Mannequin::glutDraw() {
	DrawActor(head_act);
	DrawActor(body_act);
	DrawActor(upper_arm1);
	DrawActor(upper_arm2);
	DrawActor(lower_arm1);
	DrawActor(lower_arm2);
	DrawActor(upper_leg1);
	DrawActor(upper_leg2);
	DrawActor(lower_leg1);
	DrawActor(lower_leg2);
}

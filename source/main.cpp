//
//  main.cpp
//  MRSS
//
//  Created by tengcm on 14/10/29.
//
//

#include <iostream>
#include <GLUT/GLUT.h>
#include <PxPhysicsAPI.h>
#include "main.h"

#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768

using namespace std;
using namespace physx;

// for mouse dragging
bool mouseisdown = false;
int oldx, oldy;
int roty = 30;
int rotx = 20;

// for moving perspective
float dist = -20;
float horizontal = 0;

// for initianlizing lock
bool locked = 0;
float countdown = 1000;
float elapsed = 0;
float start = 0;

// for calculating fps
float fps = 0;
int startTime = 0;
int totalFrames = 0;

// for auto swing
float max_angle = 0;
float min_angle = 0;

// modes
bool help = 0;
bool paused = 1;
bool manual = 1;

static PxFoundation				*gFoundation = NULL;
static PxPhysics				*gPhysicsSDK = NULL;
static PxDefaultCpuDispatcher	*mCpuDispatcher = NULL;
static PxDefaultErrorCallback	gDefaultErrorCallback;
static PxDefaultAllocator		gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader=PxDefaultSimulationFilterShader;
PxScene* gScene = NULL;
PxReal myTimestep = 1.0f/40.0f;

typedef struct MannequinRot {
	int mode;
	PxReal armrot_u;
	PxReal armrot_l;
	PxReal legrot_u;
	PxReal legrot_l;
} MannequinRot;

MannequinRot mode1 = {
	1,
	PxPi * -1 / 4,
	PxPi * 2 / 3,
	PxPi * -1 / 2,
	PxPi * 1 / 2
};
MannequinRot mode2 = {
	2,
	PxPi * 3 / 4,
	PxPi * -1 / 2,
	PxPi * 0 / 3,
	PxPi * -1 / 6
};

class Mannequin {
public:
	//for toggle drive
	bool active = 0;
	
	vector<PxD6Joint*> MannequinJoints;
	
	double
	head_radius,
	body_width, body_height, body_length,
	arm_width, arm_height, arm_length,
	leg_width, leg_height, leg_length;
	
	PxRigidDynamic
	*head_act, *body_act,
	*upper_arm1, *upper_arm2, *lower_arm1, *lower_arm2,
	*upper_leg1, *upper_leg2, *lower_leg1, *lower_leg2;
	
	PxD6Joint
	*j_neck,
	*j_sho_1, *j_sho_2, *j_arm_1, *j_arm_2,
	*j_leg_1, *j_leg_2, *j_kne_1, *j_kne_2;
	
	MannequinRot mode = mode1;
	
	Mannequin(double height, double shoulder_width, PxVec3 foot_position,
			  PxReal density, PxMaterial *mMaterial);
	
	PxD6JointDrive *drive;
	
	//	force = spring * (targetPosition - position) + damping * (targetVelocity - velocity)
	void DriveOn() {
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
	void DriveOff() {
		active = 0;
		drive->damping = 0.0f;
		drive->stiffness = 0.0f;
		for (int i = 1; i < MannequinJoints.size(); i++) {
			MannequinJoints[i]->setDrive(PxD6Drive::eSWING, *drive);
			MannequinJoints[i]->setDriveVelocity(PxVec3(PxZero), PxVec3(PxZero));
		}
		
	}
	
	void setMode(MannequinRot m) {
		mode = m;
		DriveOn();
	}
	
	void Unlock() {
		locked = 0;
		body_act->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
	}
	void Clear() {
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
	void glutDraw() {
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
};

// Mannequin constructor
Mannequin::Mannequin(double height, double shoulder_width, PxVec3 foot_position,
					 PxReal density, PxMaterial *mMaterial) {
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
	locked = 1;
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

class Chain {
public:
	PxArticulation *articulation;
	vector<PxArticulationLink*> ArticLinks;
	
	double seg_joint_dist;
	
	Chain(PxVec3 pos, int seg_num, PxReal seg_radius, PxReal seg_half_length,
		  PxReal density, PxMaterial *mMaterial);
	
	void Clear() {
// Individual articulation links can not be removed from the scene
// remove articulrtion directly
		gScene->removeArticulation(*articulation);
	}
	
	void glutDraw() {
		for(int i=0;i<ArticLinks.size();i++ ) {
			DrawActor(ArticLinks[i]);
		}
	}
	
};

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

class Swing {
	PxShape *shaft_stick, *seat_board;
	
public:
	PxRigidStatic *shaft;
	PxRigidDynamic *seat;
	Chain *chain1, *chain2;
	
	double str_head_height, seat_height, seg_joint_dist;
	
	Swing(PxVec3 pos, PxReal half_width, PxReal seg_num, PxReal seg_radius, PxReal seg_half_length,
		  PxReal chain_density, PxMaterial *chain_material,
		  PxReal seat_density, PxMaterial *seat_material);
	
	void Clear() {
		gScene->removeActor(*shaft);
		gScene->removeActor(*seat);
		chain1->Clear();
		delete chain1;
		chain2->Clear();
		delete chain2;
	}
	
	void glutDraw() {
		DrawActor(shaft);
		chain1->glutDraw();
		chain2->glutDraw();
		DrawActor(shaft);
		DrawActor(seat);
	}
	
// return angle by degree
	float getSwingAngle() {
		PxVec3 shaft_pos = (PxShapeExt::getGlobalPose(*shaft_stick, *shaft)).p;
		PxVec3 seat_pos = (PxShapeExt::getGlobalPose(*seat_board, *seat)).p;
		PxVec3 p = seat_pos - shaft_pos;
		return atanf(-p.z/p.y) / 3.14 * 180;
	}
};

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
		cerr<<"create actor failed!"<<endl;
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
		cerr<<"create actor failed!"<<endl;
	
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

#include <vector>
vector<PxRigidActor*> Actors;
vector<Swing*> swing;
vector<Mannequin*> M0;

int main( int argc, char** argv ) {
	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutCreateWindow("Swing");
	
	glutMouseFunc		(MouseFunc);
	glutKeyboardFunc	(KeyPressFunc);
	glutSpecialFunc		(SpecialKeyFunc);
	glutReshapeFunc		(ResizeWindow);
	glutMotionFunc		(MotionFunc);
	glutIdleFunc		(IdleFunc);
	glutDisplayFunc		(Animate);
	atexit				(Shutdown);
	
	OpenGLInit();
	InitializePhysX();
	
	glutMainLoop();
}

void MouseFunc(int button , int state, int x, int y) {
	if(button == GLUT_LEFT_BUTTON) {
		if(state == GLUT_DOWN) {
			mouseisdown=true;
		} else {
			mouseisdown=false;
		}
		oldx = x;
		oldy = y;
	}
}

void KeyPressFunc(unsigned char key, int x, int y) {
	switch ( key ) {
		case 'D':
		case 'd':
			for (int i = 0; i < M0.size(); i++) {
				if (M0[i]->active) {
					M0[i]->DriveOff();
				} else {
					M0[i]->DriveOn();
					M0[i]->setMode(mode1);
				}
			}
			break;
			
		case 'H':
		case 'h':
			help = !help;
			break;
			
		case 'M':
		case 'm':
			manual = !manual;
			max_angle = min_angle = 0;
			break;
			
		case 'P':
		case 'p':
			paused = !paused;
			break;
			
		case 'R':
		case 'r':
			Clean();
			CreateInitialActors();
			// global variable initializations
			locked = 1;
			countdown = 1000;
			elapsed = 0;
			start = glutGet(GLUT_ELAPSED_TIME);
			
			fps = 0;
			startTime = 0;
			totalFrames = 0;
			
			max_angle = 0;
			min_angle = 0;
			break;
			
		case 32:	// Space Bar
			for (int i = 0; i < M0.size(); i++) {
				if (M0[i]->mode.mode == 1) {
					M0[i]->setMode(mode2);
				} else {
					M0[i]->setMode(mode1);
				}
			}
			break;
			
		case 27:	// Escape key
			exit(1);
	}
}

void SpecialKeyFunc(int key, int x, int y) {
	switch(key) {
		case GLUT_KEY_LEFT:
			horizontal++;
			break;
		case GLUT_KEY_RIGHT:
			horizontal--;
			break;
		case GLUT_KEY_UP:
			if (dist < 0) dist++;
			break;
		case GLUT_KEY_DOWN:
			dist--;
			break;
	}
}

void ResizeWindow(int w, int h) {
	float aspectRatio;
	h = (h == 0) ? 1 : h;
	w = (w == 0) ? 1 : w;
	glViewport( 0, 0, w, h );	// View port uses whole window
	aspectRatio = (float)w/(float)h;
	
	// Set up the projection view matrix (not very well!)
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective(60, aspectRatio, 0.1f, 50.0f);
	
	// Select the Modelview matrix
	glMatrixMode( GL_MODELVIEW );
}

void MotionFunc(int x, int y) {
	if(mouseisdown==true) {
		roty += x - oldx;
		rotx += y - oldy;
		oldx = x;
		oldy = y;
	}
}

// Refresh all time
void IdleFunc() {
	glutPostRedisplay();
}

char buffer[FILENAME_MAX];
void Animate() {
	//Initial lock
	if (locked) {
		if (elapsed < countdown) {
			if (paused) {
				countdown -= elapsed;
				start = glutGet(GLUT_ELAPSED_TIME);
				elapsed = 0;
			} else {
				elapsed = glutGet(GLUT_ELAPSED_TIME) - start;
			}
		} else {
			for (int i = 0; i < M0.size(); i++) {
				M0[i]->Unlock();
				M0[i]->setMode(mode1);
			}
		}
	}
	
	//Calculate fps
	totalFrames++;
	int current = glutGet(GLUT_ELAPSED_TIME);
	if((current-startTime)>1000) {
		float elapsedTime = float(current - startTime);
		fps = ((totalFrames * 1000.0f)/ elapsedTime) ;
		startTime = current;
		totalFrames = 0;
	}
	
	//Update PhysX
	float currentAngle = swing[0]->getSwingAngle();
	if (gScene){
		if (help) {
			sprintf(buffer, "D: toggle drive\nM: manual/auto\nP: pause/resume\nR: reset\nSpace: toggle pose(manual)\nArrow Keys: move perspective\nEsc: exit\nH to return");
		} else {
			char l1[1024], l2[1024], l3[1024];
			
			sprintf(l1, "FPS: %3.2f Angle: %3.2f", fps, currentAngle);
//			PxTolerancesScale t = gPhysicsSDK->getTolerancesScale();
//			sprintf(l1, "%lf %lf %lf", t.length, t.mass, t.speed);
			
			sprintf(l2, "Drive:");
			if (M0[0]->active) {
				strcat(l2, " On  ");
			} else {
				strcat(l2, " Off  ");
			}
			if (manual) {
				strcat(l2, " Manual ");
			} else {
				strcat(l2, " Auto ");
			}
			if (paused) {
				strcat(l2, " Pause");
			} else {
				if (locked) {
					strcat(l2, " Initializing...");
				}
				StepPhysX();
			}
			
			sprintf(l3, "H for help");
			
			sprintf(buffer, "%s\n%s\n%s", l1, l2, l3);
		}
	}
	
	// Automatic Mannequin
	if (!locked && !manual) {
		if (currentAngle > max_angle) {
			max_angle = currentAngle;
		}
		if (currentAngle < min_angle) {
			min_angle = currentAngle;
		}
		float range = (max_angle - min_angle) / 6;
		
		if (currentAngle > range || currentAngle < -range) {
			M0[0]->setMode(mode2);
		} else {
			M0[0]->setMode(mode1);
		}
	}
	
	// Other GL releted stuff
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	
	glTranslatef(horizontal, -5, dist);
	glRotatef(rotx, 1, 0, 0);
	glRotatef(roty, 0, 1, 0);
	
	DrawAxes();
	DrawGrid(10);
	
	glEnable(GL_LIGHTING);
	RenderActors();
	glDisable(GL_LIGHTING);
	
	//Show text
	SetOrthoForFont();
	glColor3f(1, 1, 1);
	RenderSpacedBitmapString(20, 20, 0, GLUT_BITMAP_HELVETICA_12, buffer);
	ResetPerspectiveProjection();
	
	glutSwapBuffers();
}

void StepPhysX() {
	gScene->simulate(myTimestep);

	//...perform useful work here using previous frame's state data
	while(!gScene->fetchResults() )
	{
		// do something useful
	}
}

// Used in transforming graphical shapes to fit with result of physic simulation
void getColumnMajor(PxMat33 m, PxVec3 t, float* mat) {
	mat[0] = m.column0[0];
	mat[1] = m.column0[1];
	mat[2] = m.column0[2];
	mat[3] = 0;
	
	mat[4] = m.column1[0];
	mat[5] = m.column1[1];
	mat[6] = m.column1[2];
	mat[7] = 0;
	
	mat[8] = m.column2[0];
	mat[9] = m.column2[1];
	mat[10] = m.column2[2];
	mat[11] = 0;
	
	mat[12] = t[0];
	mat[13] = t[1];
	mat[14] = t[2];
	mat[15] = 1;
}

// Used in drawing text
void SetOrthoForFont() {
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT);
	glScalef(1, -1, 1);
	glTranslatef(0, -WINDOW_HEIGHT, 0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

// Used in drawing text, blackout if not used
void ResetPerspectiveProjection() {
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

// Used in drawing text
void RenderSpacedBitmapString(int x,
							  int y,
							  int spacing,
							  void *font,
							  char *string)
{
	char *c;
	int x1 = x;
	int y1 = y;
	for (c = string; *c != '\0'; c++) {
		if (*c == '\n') {
			x1 = x;
			y1 += 20;
		} else {
			glRasterPos2i(x1,y1);
			glutBitmapCharacter(font, *c);
		}
		x1 += glutBitmapWidth(font,*c) + spacing;
	}
}

void DrawAxes() {
	enum l {X, Y, Z};
	char label[3][2] = {"X", "Y", "Z"};
	
	//To prevent the view from disturbed on repaint
	//this push matrix call stores the current matrix state
	//and restores it once we are done with the arrow rendering
	glPushMatrix();
	
	glColor3f(0,0,1);
	glPushMatrix();
	glTranslatef(0,0, 0.8f);
	glutSolidCone(0.0325,0.2, 4,1);
	//Draw label
	glTranslatef(0,0.0625,0.225f);
	RenderSpacedBitmapString(0,0,0,GLUT_BITMAP_HELVETICA_10, label[Z]);
	glPopMatrix();
	glutSolidCone(0.0225,1, 4,1);
	
	glColor3f(1,0,0);
	glRotatef(90,0,1,0);
	glPushMatrix();
	glTranslatef(0,0,0.8f);
	glutSolidCone(0.0325,0.2, 4,1);
	//Draw label
	glTranslatef(0,0.0625,0.225f);
	RenderSpacedBitmapString(0,0,0,GLUT_BITMAP_HELVETICA_10, label[X]);
	glPopMatrix();
	glutSolidCone(0.0225,1, 4,1);
	
	glColor3f(0,1,0);
	glRotatef(90,-1,0,0);
	glPushMatrix();
	glTranslatef(0,0, 0.8f);
	glutSolidCone(0.0325,0.2, 4,1);
	//Draw label
	glTranslatef(0,0.0625,0.225f);
	RenderSpacedBitmapString(0,0,0,GLUT_BITMAP_HELVETICA_10, label[Y]);
	glPopMatrix();
	glutSolidCone(0.0225,1, 4,1);
	
	glPopMatrix();
}
void DrawGrid(int GRID_SIZE) {
	glBegin(GL_LINES);
	glColor3f(0.75f, 0.75f, 0.75f);
	for(int i=-GRID_SIZE;i<=GRID_SIZE;i++)
	{
		glVertex3f((float)i,0,(float)-GRID_SIZE);
		glVertex3f((float)i,0,(float)GRID_SIZE);
		
		glVertex3f((float)-GRID_SIZE,0,(float)i);
		glVertex3f((float)GRID_SIZE,0,(float)i);
	}
	glEnd();
}


void DrawActor(PxRigidActor* actor) {
	PxU32 nShapes = actor->getNbShapes();
	PxShape **shapes = new PxShape*[nShapes];
	
	actor->getShapes(shapes, nShapes);
	while (nShapes--) {
		DrawShape(shapes[nShapes], actor);
	}
	delete [] shapes;
}

void RenderActors()  {
	// Render all the actors in the scene
	for (int i = 0; i < M0.size(); i++) {
		M0[i]->glutDraw();
	}
	for (int i = 0; i < swing.size(); i++) {
		swing[i]->glutDraw();
	}
	for (int i = 0; i < Actors.size(); i++) {
		DrawActor(Actors[i]);
	}
}

void DrawShape(PxShape* shape, PxRigidActor *actor) {
	PxGeometryType::Enum type = shape->getGeometryType();
	switch(type)
	{
		case PxGeometryType::eBOX:
			glColor3f(0.5,0.5,0.5);
			DrawBox(shape, actor);
			break;
			
		case PxGeometryType::eSPHERE:
			glColor3f(0.5,0.5,0.5);
			DrawSphere(shape, actor);
			break;
			
		case PxGeometryType::eCAPSULE:
			glColor3f(0.5,0.5,0.5);
			DrawCapsule(shape, actor);
			break;
			
		default:
			cerr<<"Shape not supported."<<type<<endl;
	}
}

void DrawBox(PxShape* pShape, PxRigidActor *actor) {
	PxTransform pT = PxShapeExt::getGlobalPose(*pShape, *actor);
	PxBoxGeometry bg;
	pShape->getBoxGeometry(bg);
	PxMat33 m = PxMat33(pT.q );
	float mat[16];
	getColumnMajor(m, pT.p, mat);
	glPushMatrix();
	glMultMatrixf(mat);
	glScalef(bg.halfExtents.x * 2, bg.halfExtents.y * 2, bg.halfExtents.z * 2);
	glutSolidCube(1);
	glPopMatrix();
}
void DrawSphere(PxShape* pShape, PxRigidActor *actor) {
	PxTransform pT = PxShapeExt::getGlobalPose(*pShape, *actor);
	PxSphereGeometry bg;
	pShape->getSphereGeometry(bg);
	PxMat33 m = PxMat33(pT.q );
	float mat[16];
	getColumnMajor(m,pT.p, mat);
	glPushMatrix();
	glMultMatrixf(mat);
	glutSolidSphere(bg.radius, 20, 20);
	glPopMatrix();
}
void DrawCapsule(PxShape* pShape, PxRigidActor *actor) {
	PxTransform pT = PxShapeExt::getGlobalPose(*pShape, *actor);
	PxCapsuleGeometry bg;
	pShape->getCapsuleGeometry(bg);
	PxMat33 m = PxMat33(pT.q);
	float mat[16];
	getColumnMajor(m, pT.p, mat);
	
	glPushMatrix();
	
	glMultMatrixf(mat);
	glPushMatrix();

	glTranslatef(bg.halfHeight, 0, 0);
	glRotatef(90, 0, 1, 0);
	glutSolidSphere(bg.radius, 20, 20);

	glPopMatrix();
	glTranslatef(-bg.halfHeight, 0, 0);
	glRotatef(90, 0, 1, 0);
	glutSolidSphere(bg.radius, 20, 20);
//cylinder drawing starts from the bottom;
	gluCylinder(gluNewQuadric(), bg.radius, bg.radius, bg.halfHeight * 2, 20, 20);
	
	glPopMatrix();
}


void Shutdown() {
	Clean();
	gScene->release();
	mCpuDispatcher->release();
	gPhysicsSDK->release();
	gFoundation->release();
}

void Clean() {
	for (int i = 0; i < Actors.size(); i++) {
		gScene->removeActor(*Actors[i]);
	}
	Actors.clear();
	
	for (int i = 0; i < M0.size(); i++) {
		M0[i]->Clear();
		delete M0[i];
	}
	M0.clear();
	
	for (int i = 0; i < swing.size(); i++) {
		swing[i]->Clear();
		delete swing[i];
	}
	swing.clear();
}

void OpenGLInit(){
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	
	GLfloat ambient[4]={0.25,0.25,0.25,0.25};
	GLfloat diffuse[4]={1,1,1,1};
	
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glEnable(GL_COLOR_MATERIAL);
	
	glDisable(GL_LIGHTING);
	
}

void InitializePhysX() {
	gFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
	gPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale());
	if(gPhysicsSDK == NULL) {
		cerr<<"Error creating PhysX3 device."<<endl;
		cerr<<"Exiting..."<<endl;
		exit(1);
	}
	
	//Create the scene
	PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());
	sceneDesc.gravity=PxVec3(0.0f, -9.8f, 0.0f);
	
	if(!sceneDesc.cpuDispatcher) {
		mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
		if(!mCpuDispatcher)
			cerr<<"PxDefaultCpuDispatcherCreate failed!"<<endl;
		sceneDesc.cpuDispatcher = mCpuDispatcher;
	}
	if(!sceneDesc.filterShader)
		sceneDesc.filterShader  = gDefaultFilterShader;
	
	
	gScene = gPhysicsSDK->createScene(sceneDesc);
	if (!gScene)
		cerr<<"createScene failed!"<<endl;
	
	CreateInitialActors();
}

void CreateInitialActors() {
	// Create ground plane
	PxMaterial* ground = gPhysicsSDK->createMaterial(0.25, 0.1, 0.1);
	PxTransform pose = PxTransform(PxVec3(0.0f, 0.0f, 0.0f),PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));
	
	PxRigidStatic *plane = gPhysicsSDK->createRigidStatic(pose);
	if (!plane)
		cerr<<"create plane failed!"<<endl;
	
	PxShape* shape = plane->createShape(PxPlaneGeometry(), *ground);
	if (!shape)
		cerr<<"create shape failed!"<<endl;
	gScene->addActor(*plane);
	Actors.push_back(plane);
	
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
	
	
	Swing *s = new Swing(PxVec3(0, swing_height, 0), half_width, seg_num, seg_radius, seg_half_length,
					  iron_density, iron, wood_density, wood);
	swing.push_back(s);
	
	Mannequin *m = new Mannequin(mannequin_height, shoulder_width,
								 PxVec3(0, swing[0]->seat_height + 0.2, 0), body_density, body);
	M0.push_back(m);
	
	// Make Mannequin grasp the chain
	int hand_seg_num = 5;
	PxTransform arm_hand(PxVec3(-M0[0]->arm_length / 2, 0, 0));
	PxD6Joint *j;
	
	j = PxD6JointCreate(*gPhysicsSDK, M0[0]->lower_arm1, arm_hand,
						swing[0]->chain1->ArticLinks[hand_seg_num], PxTransform(PxVec3(0)));
	j->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	
	j = PxD6JointCreate(*gPhysicsSDK, M0[0]->lower_arm2, arm_hand,
						swing[0]->chain2->ArticLinks[hand_seg_num], PxTransform(PxVec3(0)));
	j->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	
	PxTransform leg_foot(PxVec3(-((M0[0]->leg_length) / 2 + 0.2), 0, 0));
	PxVec3 seat_foot((M0[0]->body_width - M0[0]->leg_width - 0.1) / 2, 0, 0);
	
	j = PxD6JointCreate(*gPhysicsSDK, M0[0]->lower_leg1, leg_foot,
						swing[0]->seat, PxTransform(seat_foot));
	j->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	
	j = PxD6JointCreate(*gPhysicsSDK, M0[0]->lower_leg2, leg_foot,
						swing[0]->seat, PxTransform(-seat_foot));
	j->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	
}

PxQuat QuatRotate(PxVec3 axis, PxReal rot) {
	PxQuat ret;
	axis /= axis.magnitude();
	ret.w = cos(rot / 2);
	ret.x = sin(rot / 2) * axis.x;
	ret.y = sin(rot / 2) * axis.y;
	ret.z = sin(rot / 2) * axis.z;
	return ret;
}
PxVec3 VecRotate (PxReal theta, PxReal phi) {
	PxVec3 ret;
	ret.x = PxSin(theta) * PxCos(phi);
	ret.y = PxSin(theta) * PxSin(phi);
	ret.z = PxCos(theta);
	return ret;
}

PxRigidDynamic* CreateBox (PxVec3 pos, PxVec3 axis, PxReal rot, PxVec3 dimension,
						   PxReal density, PxMaterial* mMaterial) {
	PxTransform transform(pos, QuatRotate(axis, rot));
	PxBoxGeometry geometry(dimension);
	PxRigidDynamic *actor = PxCreateDynamic(*gPhysicsSDK, transform, geometry, *mMaterial, density);
	if (!actor)
		cerr<<"create actor failed!"<<endl;
	actor->setAngularDamping(0.75);
	actor->setLinearVelocity(PxVec3(0, 0, 0));
	gScene->addActor(*actor);
	return actor;
}
PxRigidDynamic* CreateSphere(PxVec3 pos, PxReal radius,
							 PxReal density, PxMaterial* mMaterial) {
	// Add a single-shape actor to the scene
	PxTransform transform(pos, PxQuat(PxIdentity));
	PxSphereGeometry geometry(radius);
	
	PxRigidDynamic *actor = PxCreateDynamic(*gPhysicsSDK, transform, geometry, *mMaterial, density);
	if (!actor)
		cerr<<"create actor failed!"<<endl;
	actor->setAngularDamping(0.75);
	actor->setLinearVelocity(PxVec3(0,0,0));
	gScene->addActor(*actor);
	return actor;
}
PxRigidDynamic* CreateCapsule(PxVec3 pos, PxVec3 axis, PxReal rot, PxReal radius, PxReal half_height,
							  PxReal density, PxMaterial *mMaterial) {
	PxTransform transform(pos, QuatRotate(axis, rot));
	PxCapsuleGeometry geometry(radius, half_height);
	PxRigidDynamic *actor = PxCreateDynamic(*gPhysicsSDK, transform, geometry, *mMaterial, density);
	if (!actor)
		cerr<<"create actor failed!"<<endl;
	actor->setAngularDamping(0.75);
	actor->setLinearVelocity(PxVec3(0, 0, 0));
	gScene->addActor(*actor);
	return actor;
}

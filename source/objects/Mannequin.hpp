//
//  Mannequin.hpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#ifndef Mannequin_hpp
#define Mannequin_hpp

#include <vector>

#include "main.hpp"
#include "PhysX.hpp"
#include "GlobalState.hpp"

typedef struct MannequinRot {
	int mode;
	PxReal armrot_u;
	PxReal armrot_l;
	PxReal legrot_u;
	PxReal legrot_l;
} MannequinRot;

class Mannequin {
public:
	//for toggle drive
	bool active = 0;
	
	std::vector<PxD6Joint*> MannequinJoints;
	
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
	
	MannequinRot mode;
	
	Mannequin(double height, double shoulder_width, PxVec3 foot_position,
			  PxReal density, PxMaterial *mMaterial);
	
	PxD6JointDrive *drive;
	
	void DriveOn();
	void DriveOff();
	
	void setMode(MannequinRot m);
	
	void Unlock();
	void Clear();
	void glutDraw();
};


#endif /* Mannequin_hpp */

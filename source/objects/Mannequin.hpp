//
//  Mannequin.hpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#ifndef Mannequin_hpp
#define Mannequin_hpp

#include "BaseObject.hpp"

#include <vector>
#include <map>

#include "PhysX.hpp"
#include "GlobalState.hpp"
#include "SingleObject.hpp"

enum class MANNEQUIN {
	HEAD = 0, BODY,
	U_ARM_L, U_ARM_R, L_ARM_L, L_ARM_R,
	U_LEG_L, U_LEG_R, L_LEG_L, L_LEG_R,
	BODY_PART_NUM
};

struct MannequinRot {
	PxReal armrot_u;
	PxReal armrot_l;
	PxReal legrot_u;
	PxReal legrot_l;
};

class Mannequin : public BaseObject
{
public:
	Mannequin(double height, double shoulder_width, PxVec3 foot_position,
			  PxReal density, PxMaterial *mMaterial);

	~Mannequin();

	void addToPxScene(PxScene *scene) override;
	void removeFromPxScene(PxScene *scene) override;
	void draw() override;

	bool isLocked() { return m_locked; }

	void driveOn();
	void driveOff();
	bool driveIsOn() { return m_drive_on; }

	void setMode(MannequinRot m);

	void unlock();
	void clear();

	double arm_length, leg_length, body_width, leg_width;
	PxRigidDynamic *lower_arm1, *lower_arm2, *lower_leg1, *lower_leg2;

private:
	bool m_locked;
	bool m_drive_on;

	MannequinRot m_rot;

	std::unique_ptr<PxD6JointDrive> m_D6_drive;
	std::map<MANNEQUIN, SingleObject> m_body_parts;
	std::vector<PxD6Joint*> MannequinJoints;

	double
	head_radius,
	//body_width,
	body_height, body_length,
	arm_width, arm_height, //arm_length,
	//leg_width,
	leg_height;// leg_length;

	PxRigidDynamic
	*head_act, *body_act,
	*upper_arm1, *upper_arm2,// *lower_arm1, *lower_arm2,
	*upper_leg1, *upper_leg2; // *lower_leg1, *lower_leg2;

	PxD6Joint
	*j_neck,
	*j_sho_1, *j_sho_2, *j_arm_1, *j_arm_2,
	*j_leg_1, *j_leg_2, *j_kne_1, *j_kne_2;

};


#endif /* Mannequin_hpp */

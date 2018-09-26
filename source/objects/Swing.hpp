//
//  Swing.hpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#ifndef Swing_hpp
#define Swing_hpp

#include "main.hpp"
#include "PhysX.hpp"
#include "Chain.hpp"

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


#endif /* Swing_hpp */

//
//  Swing.hpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#ifndef Swing_hpp
#define Swing_hpp

#include "BaseObject.hpp"

#include <vector>

#include "PhysX.hpp"
#include "Chain.hpp"
#include "SingleObject.hpp"

class Swing : public BaseObject
{
public:
	Swing(PxVec3 pos, PxReal half_width, PxReal seg_num, PxReal seg_radius, PxReal seg_half_length,
		  PxReal chain_density, PxMaterial *chain_material,
		  PxReal seat_density, PxMaterial *seat_material);

	~Swing();

	void addToPxScene(PxScene *scene) override;
	void removeFromPxScene(PxScene *scene) override;
	void draw() override;

	// return angle by degree
	float getAngle();

	Chain* getChain(int i) { return m_chains[i].get(); }

	double seat_height;
	PxRigidDynamic *seat;

private:
	std::unique_ptr<Chain> m_chains[2];
	std::unique_ptr<SingleObject> m_shaft;
	std::unique_ptr<SingleObject> m_seat;

	double str_head_height, seg_joint_dist;

	PxRigidStatic *shaft;

	PxShape *shaft_stick, *seat_board;
};


#endif /* Swing_hpp */

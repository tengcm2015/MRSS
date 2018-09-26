//
//  Chain.hpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#ifndef Chain_hpp
#define Chain_hpp

#include <vector>

#include "main.hpp"
#include "PhysX.hpp"

class Chain {
public:
	PxArticulation *articulation;
	std::vector<PxArticulationLink*> ArticLinks;
	
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


#endif /* Chain_hpp */

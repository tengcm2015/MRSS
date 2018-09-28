//
//  Chain.hpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#ifndef Chain_hpp
#define Chain_hpp

#include "BaseObject.hpp"

#include <vector>

#include "PhysX.hpp"
#include "SingleObject.hpp"

class Chain : public BaseObject
{
public:
	Chain(PxVec3 pos, int seg_num, PxReal seg_radius, PxReal seg_half_length,
		  PxReal density, PxMaterial *mMaterial);

	~Chain();

	void addToPxScene(PxScene *scene) override;
	void removeFromPxScene(PxScene *scene) override;
	void draw() override;

	SingleObject* getLink(int i) { return &m_links[i]; }

private:
	PxArticulation *m_articulation;
//	std::vector<PxArticulationLink*> ArticLinks;
	std::vector<SingleObject> m_links;
};


#endif /* Chain_hpp */

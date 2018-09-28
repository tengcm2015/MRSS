//
//  SingleObject.hpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#ifndef SingleObject_hpp
#define SingleObject_hpp

#include "PhysX.hpp"
#include "BaseObject.hpp"

class SingleObject : public BaseObject
{
public:
	SingleObject(PxRigidActor* actor);

	// move constructors
	SingleObject(SingleObject&&);
	SingleObject& operator=(SingleObject&&);

	~SingleObject();

	void addToPxScene(PxScene *scene) override;
	void removeFromPxScene(PxScene *scene) override;
	void draw() override;

	PxRigidActor* getActor() { return m_actor; }

private:
	PxRigidActor *m_actor;
};

#endif /* SingleObject_hpp */

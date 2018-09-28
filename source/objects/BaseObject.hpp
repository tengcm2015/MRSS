//
//  BaseObject.hpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#ifndef BaseObject_hpp
#define BaseObject_hpp

#include <vector>
#include "PhysX.hpp"

class BaseObject
{
public:
	// copy disallowed
	BaseObject(const BaseObject&) = delete;
	BaseObject& operator=(const BaseObject&) = delete;

	virtual void addToPxScene(PxScene *scene) = 0;
	virtual void removeFromPxScene(PxScene *scene) = 0;
	virtual void draw() = 0;

protected:
	BaseObject() {}
	virtual ~BaseObject() {}
};

#endif /* BaseObject_hpp */

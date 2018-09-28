//
//  GlobalState.hpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#ifndef GlobalState_hpp
#define GlobalState_hpp

#include "PhysX.hpp"

struct GlobalState {
	// for mouse dragging
	bool mouseisdown = false;
	int oldx, oldy;
	int roty = 30;
	int rotx = 20;

	// for moving perspective
	float dist = -20;
	float horizontal = 0;

	// for initianlizing lock
	float countdown = 1000;
	float elapsed = 0;
	float start = 0;

	// for calculating fps
	float fps = 0;
	int startTime = 0;
	int totalFrames = 0;

	// modes
	bool help = false;
	bool paused = true;
	bool manual = false;

	PxReal myTimestep = 1.0f/40.0f;
};

extern struct GlobalState globalState;

#endif /* GlobalState_hpp */

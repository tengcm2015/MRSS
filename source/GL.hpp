//
//  GL.hpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#ifndef GL_hpp
#define GL_hpp

#include <GLUT/GLUT.h>

#include <memory>

class GL {
public:
	GL();
	~GL();
};

extern std::unique_ptr<GL> gl;

#endif /* GL_hpp */

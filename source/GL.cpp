//
//  GL.cpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#include "GL.hpp"

GL *gl;

GL::GL()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	
	GLfloat ambient[4]={0.25,0.25,0.25,0.25};
	GLfloat diffuse[4]={1,1,1,1};
	
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glEnable(GL_COLOR_MATERIAL);
	
	glDisable(GL_LIGHTING);
	
}

GL::~GL()
{
}

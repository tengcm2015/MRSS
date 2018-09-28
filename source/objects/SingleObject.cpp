//
//  SingleObject.cpp
//  MRSS
//
//  Created by tengcm on 2018/9/26.
//

#include "SingleObject.hpp"

#include <iostream>
#include "GL.hpp"

SingleObject::SingleObject(PxRigidActor *actor) :
	BaseObject(),
	m_actor(actor)
{
}

// Move constructor
// Transfer ownership of a.m_actor to m_actor
SingleObject::SingleObject(SingleObject&& a) :
	m_actor(a.m_actor)
{
	a.m_actor = nullptr;
}

// Move assignment
// Transfer ownership of a.m_actor to m_actor
SingleObject& SingleObject::operator=(SingleObject&& a)
{
	// Self-assignment detection
	if (&a == this)
		return *this;

	// Release any resource we're holding
	m_actor->release();

	// Transfer ownership of a.m_ptr to m_ptr
	m_actor = a.m_actor;
	a.m_actor = nullptr;

	return *this;
}

SingleObject::~SingleObject()
{
	if (m_actor)
		m_actor->release();
}

void SingleObject::addToPxScene(PxScene *scene) {
	scene->addActor(*m_actor);
}

void SingleObject::removeFromPxScene(PxScene *scene) {
	scene->removeActor(*m_actor);
}

// Used in transforming graphical shapes to fit with result of physic simulation
static void getColumnMajor(PxMat33 m, PxVec3 t, float* mat) {
	mat[0] = m.column0[0];
	mat[1] = m.column0[1];
	mat[2] = m.column0[2];
	mat[3] = 0;

	mat[4] = m.column1[0];
	mat[5] = m.column1[1];
	mat[6] = m.column1[2];
	mat[7] = 0;

	mat[8] = m.column2[0];
	mat[9] = m.column2[1];
	mat[10] = m.column2[2];
	mat[11] = 0;

	mat[12] = t[0];
	mat[13] = t[1];
	mat[14] = t[2];
	mat[15] = 1;
}

static void DrawBox(PxShape* pShape, PxRigidActor *actor) {
	PxTransform pT = PxShapeExt::getGlobalPose(*pShape, *actor);
	PxBoxGeometry bg;
	pShape->getBoxGeometry(bg);
	PxMat33 m = PxMat33(pT.q);
	float mat[16];
	getColumnMajor(m, pT.p, mat);
	glPushMatrix();
	glMultMatrixf(mat);
	glScalef(bg.halfExtents.x * 2, bg.halfExtents.y * 2, bg.halfExtents.z * 2);
	glutSolidCube(1);
	glPopMatrix();
}

static void DrawSphere(PxShape* pShape, PxRigidActor *actor) {
	PxTransform pT = PxShapeExt::getGlobalPose(*pShape, *actor);
	PxSphereGeometry bg;
	pShape->getSphereGeometry(bg);
	PxMat33 m = PxMat33(pT.q);
	float mat[16];
	getColumnMajor(m,pT.p, mat);
	glPushMatrix();
	glMultMatrixf(mat);
	glutSolidSphere(bg.radius, 20, 20);
	glPopMatrix();
}

static void DrawCapsule(PxShape* pShape, PxRigidActor *actor) {
	PxTransform pT = PxShapeExt::getGlobalPose(*pShape, *actor);
	PxCapsuleGeometry bg;
	pShape->getCapsuleGeometry(bg);
	PxMat33 m = PxMat33(pT.q);
	float mat[16];
	getColumnMajor(m, pT.p, mat);

	glPushMatrix();

	glMultMatrixf(mat);
	glPushMatrix();

	glTranslatef(bg.halfHeight, 0, 0);
	glRotatef(90, 0, 1, 0);
	glutSolidSphere(bg.radius, 20, 20);

	glPopMatrix();
	glTranslatef(-bg.halfHeight, 0, 0);
	glRotatef(90, 0, 1, 0);
	glutSolidSphere(bg.radius, 20, 20);
	//cylinder drawing starts from the bottom;
	gluCylinder(gluNewQuadric(), bg.radius, bg.radius, bg.halfHeight * 2, 20, 20);

	glPopMatrix();
}

static void DrawShape(PxShape* shape, PxRigidActor *actor) {
	PxGeometryType::Enum type = shape->getGeometryType();
	switch(type)
	{
		case PxGeometryType::eSPHERE:
			glColor3f(0.5,0.5,0.5);
			DrawSphere(shape, actor);
			break;

		case PxGeometryType::ePLANE:
			/* Do nothing */
			break;

		case PxGeometryType::eCAPSULE:
			glColor3f(0.5,0.5,0.5);
			DrawCapsule(shape, actor);
			break;

		case PxGeometryType::eBOX:
			glColor3f(0.5,0.5,0.5);
			DrawBox(shape, actor);
			break;

		case PxGeometryType::eCONVEXMESH:
		case PxGeometryType::eTRIANGLEMESH:
		case PxGeometryType::eHEIGHTFIELD:
			/* Shape not supported */
			break;

		default:
			throw "Invalid Geometry Type";
	}
}


void SingleObject::draw () {
	PxU32 nShapes = m_actor->getNbShapes();
	PxShape **shapes = new PxShape*[nShapes];

	m_actor->getShapes(shapes, nShapes);
	while (nShapes--) {
		DrawShape(shapes[nShapes], m_actor);
	}
	delete [] shapes;
}

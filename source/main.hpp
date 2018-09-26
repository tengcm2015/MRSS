//
//  main.hpp
//  MRSS
//
//  Created by tengcm on 14/10/29.
//
//

#ifndef main_hpp
#define main_hpp

#include "PhysX.hpp"

#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768

extern PxPhysics *gPhysicsSDK;
extern PxScene *gScene;

void MouseFunc(int button , int state, int x, int y);

void KeyPressFunc(unsigned char key, int x, int y);

void SpecialKeyFunc(int key, int x, int y);

void ResizeWindow(int w, int h);

void MotionFunc(int x, int y);

void IdleFunc();


void Animate();

void StepPhysX();

void getColumnMajor(PxMat33, PxVec3, float*);
void SetOrthoForFont();
void ResetPerspectiveProjection();
void RenderSpacedBitmapString(int ,int ,int ,void* , char*);
void DrawAxes();
void DrawGrid(int);

void RenderActors();
void DrawActor(PxRigidActor* actor);
void DrawShape(PxShape* shape, PxRigidActor *actor);
void DrawBox(PxShape*, PxRigidActor*);
void DrawSphere(PxShape* pShape, PxRigidActor *actor);
void DrawCapsule(PxShape* pShape, PxRigidActor *actor);


void Shutdown();
void Clean();

void CreateInitialActors();

PxQuat QuatRotate (PxVec3 axis, PxReal rot);
PxVec3 VecRotate (PxReal theta, PxReal phi);

PxRigidDynamic* CreateBox (PxVec3 pos, PxVec3 axis, PxReal rot,
						   PxVec3 dimension, PxReal density, PxMaterial* mMaterial);
PxRigidDynamic* CreateSphere(PxVec3 pos, PxReal radius,
							 PxReal density, PxMaterial* mMaterial);
PxRigidDynamic* CreateCapsule(PxVec3 pos, PxVec3 axis, PxReal rot,
							  PxReal radius, PxReal half_height,
							  PxReal density, PxMaterial *mMaterial);

#endif /* main_hpp */

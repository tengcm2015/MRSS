//
//  main.cpp
//  MRSS
//
//  Created by tengcm on 14/10/29.
//
//

#include "main.hpp"

#include <iostream>
#include <vector>

#include "GlobalState.hpp"
#include "PhysX.hpp"
#include "GL.hpp"
#include "objects/Chain.hpp"
#include "objects/Swing.hpp"
#include "objects/Mannequin.hpp"

using namespace std;


PxPhysics *gPhysicsSDK = nullptr;
PxScene *gScene = nullptr;

vector<PxRigidActor*> Actors;
vector<Swing*> swing;
vector<Mannequin*> M0;

MannequinRot mode1 = {
	1,
	PxPi * -1 / 4,
	PxPi * 2 / 3,
	PxPi * -1 / 2,
	PxPi * 1 / 2
};
MannequinRot mode2 = {
	2,
	PxPi * 3 / 4,
	PxPi * -1 / 2,
	PxPi * 0 / 3,
	PxPi * -1 / 6
};


int main( int argc, char** argv ) {
	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutCreateWindow("More Realistic Swing Simulator (MRSS)");
	
	glutMouseFunc		(MouseFunc);
	glutKeyboardFunc	(KeyPressFunc);
	glutSpecialFunc		(SpecialKeyFunc);
	glutReshapeFunc		(ResizeWindow);
	glutMotionFunc		(MotionFunc);
	glutIdleFunc		(IdleFunc);
	glutDisplayFunc		(Animate);
	atexit				(Shutdown);
	
	gl = new GL();
	
	physicsEngine = new PhysX();
	gScene = physicsEngine->createPxScene();
	gPhysicsSDK = physicsEngine->getPhysics();
	CreateInitialActors();
	
	glutMainLoop();
}

void MouseFunc(int button , int state, int x, int y) {
	if(button == GLUT_LEFT_BUTTON) {
		if(state == GLUT_DOWN) {
			globalState.mouseisdown=true;
		} else {
			globalState.mouseisdown=false;
		}
		globalState.oldx = x;
		globalState.oldy = y;
	}
}

void KeyPressFunc(unsigned char key, int x, int y) {
	switch ( key ) {
		case 'D':
		case 'd':
			for (int i = 0; i < M0.size(); i++) {
				if (M0[i]->active) {
					M0[i]->DriveOff();
				} else {
					M0[i]->DriveOn();
					M0[i]->setMode(mode1);
				}
			}
			break;
			
		case 'H':
		case 'h':
			globalState.help = !globalState.help;
			break;
			
		case 'M':
		case 'm':
			globalState.manual = !globalState.manual;
			globalState.max_angle = globalState.min_angle = 0;
			break;
			
		case 'P':
		case 'p':
			globalState.paused = !globalState.paused;
			break;
			
		case 'R':
		case 'r':
			Clean();
			CreateInitialActors();
			// global variable initializations
			globalState.locked = 1;
			globalState.countdown = 1000;
			globalState.elapsed = 0;
			globalState.start = glutGet(GLUT_ELAPSED_TIME);
			
			globalState.fps = 0;
			globalState.startTime = 0;
			globalState.totalFrames = 0;
			
			globalState.max_angle = 0;
			globalState.min_angle = 0;
			break;
			
		case 32:	// Space Bar
			for (int i = 0; i < M0.size(); i++) {
				if (M0[i]->mode.mode == 1) {
					M0[i]->setMode(mode2);
				} else {
					M0[i]->setMode(mode1);
				}
			}
			break;
			
		case 27:	// Escape key
			exit(1);
	}
}

void SpecialKeyFunc(int key, int x, int y) {
	switch(key) {
		case GLUT_KEY_LEFT:
			globalState.horizontal++;
			break;
		case GLUT_KEY_RIGHT:
			globalState.horizontal--;
			break;
		case GLUT_KEY_UP:
			if (globalState.dist < 0) globalState.dist++;
			break;
		case GLUT_KEY_DOWN:
			globalState.dist--;
			break;
	}
}

void ResizeWindow(int w, int h) {
	float aspectRatio;
	h = (h == 0) ? 1 : h;
	w = (w == 0) ? 1 : w;
	glViewport( 0, 0, w, h );	// View port uses whole window
	aspectRatio = (float)w/(float)h;
	
	// Set up the projection view matrix (not very well!)
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective(60, aspectRatio, 0.1f, 50.0f);
	
	// Select the Modelview matrix
	glMatrixMode( GL_MODELVIEW );
}

void MotionFunc(int x, int y) {
	if(globalState.mouseisdown) {
		globalState.roty += x - globalState.oldx;
		globalState.rotx += y - globalState.oldy;
		globalState.oldx = x;
		globalState.oldy = y;
	}
}

// Refresh all time
void IdleFunc() {
	glutPostRedisplay();
}

char buffer[FILENAME_MAX];
void Animate() {
	//Initial lock
	if (globalState.locked) {
		if (globalState.elapsed < globalState.countdown) {
			if (globalState.paused) {
				globalState.countdown -= globalState.elapsed;
				globalState.start = glutGet(GLUT_ELAPSED_TIME);
				globalState.elapsed = 0;
			} else {
				globalState.elapsed = glutGet(GLUT_ELAPSED_TIME) - globalState.start;
			}
		} else {
			for (int i = 0; i < M0.size(); i++) {
				M0[i]->Unlock();
				M0[i]->setMode(mode1);
			}
		}
	}
	
	//Calculate fps
	globalState.totalFrames++;
	int current = glutGet(GLUT_ELAPSED_TIME);
	if((current-globalState.startTime)>1000) {
		float elapsedTime = float(current - globalState.startTime);
		globalState.fps = ((globalState.totalFrames * 1000.0f)/ elapsedTime) ;
		globalState.startTime = current;
		globalState.totalFrames = 0;
	}
	
	//Update PhysX
	float currentAngle = swing[0]->getSwingAngle();
	if (gScene){
		if (globalState.help) {
			sprintf(buffer, "D: toggle drive\nM: manual/auto\nP: pause/resume\nR: reset\nSpace: toggle pose(manual)\nArrow Keys: move perspective\nEsc: exit\nH to return");
		} else {
			char l1[1024], l2[1024], l3[1024];
			
			sprintf(l1, "FPS: %3.2f Angle: %3.2f", globalState.fps, currentAngle);
//			PxTolerancesScale t = gPhysicsSDK->getTolerancesScale();
//			sprintf(l1, "%lf %lf %lf", t.length, t.mass, t.speed);
			
			sprintf(l2, "Drive:");
			if (M0[0]->active) {
				strcat(l2, " On  ");
			} else {
				strcat(l2, " Off  ");
			}
			if (globalState.manual) {
				strcat(l2, " Manual ");
			} else {
				strcat(l2, " Auto ");
			}
			if (globalState.paused) {
				strcat(l2, " Pause");
			} else {
				if (globalState.locked) {
					strcat(l2, " Initializing...");
				}
				StepPhysX();
			}
			
			sprintf(l3, "H for help");
			
			sprintf(buffer, "%s\n%s\n%s", l1, l2, l3);
		}
	}
	
	// Automatic Mannequin
	if (!globalState.locked && !globalState.manual) {
		if (currentAngle > globalState.max_angle) {
			globalState.max_angle = currentAngle;
		}
		if (currentAngle < globalState.min_angle) {
			globalState.min_angle = currentAngle;
		}
		float range = (globalState.max_angle - globalState.min_angle) / 6;
		
		if (currentAngle > range || currentAngle < -range) {
			M0[0]->setMode(mode2);
		} else {
			M0[0]->setMode(mode1);
		}
	}
	
	// Other GL releted stuff
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	
	glTranslatef(globalState.horizontal, -5, globalState.dist);
	glRotatef(globalState.rotx, 1, 0, 0);
	glRotatef(globalState.roty, 0, 1, 0);
	
	DrawAxes();
	DrawGrid(10);
	
	glEnable(GL_LIGHTING);
	RenderActors();
	glDisable(GL_LIGHTING);
	
	//Show text
	SetOrthoForFont();
	glColor3f(1, 1, 1);
	RenderSpacedBitmapString(20, 20, 0, GLUT_BITMAP_HELVETICA_12, buffer);
	ResetPerspectiveProjection();
	
	glutSwapBuffers();
}

void StepPhysX() {
	gScene->simulate(globalState.myTimestep);

	//...perform useful work here using previous frame's state data
	while(!gScene->fetchResults() )
	{
		// do something useful
	}
}

// Used in transforming graphical shapes to fit with result of physic simulation
void getColumnMajor(PxMat33 m, PxVec3 t, float* mat) {
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

// Used in drawing text
void SetOrthoForFont() {
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT);
	glScalef(1, -1, 1);
	glTranslatef(0, -WINDOW_HEIGHT, 0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

// Used in drawing text, blackout if not used
void ResetPerspectiveProjection() {
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

// Used in drawing text
void RenderSpacedBitmapString(int x,
							  int y,
							  int spacing,
							  void *font,
							  char *string)
{
	char *c;
	int x1 = x;
	int y1 = y;
	for (c = string; *c != '\0'; c++) {
		if (*c == '\n') {
			x1 = x;
			y1 += 20;
		} else {
			glRasterPos2i(x1,y1);
			glutBitmapCharacter(font, *c);
		}
		x1 += glutBitmapWidth(font,*c) + spacing;
	}
}

void DrawAxes() {
	enum l {X, Y, Z};
	char label[3][2] = {"X", "Y", "Z"};
	
	//To prevent the view from disturbed on repaint
	//this push matrix call stores the current matrix state
	//and restores it once we are done with the arrow rendering
	glPushMatrix();
	
	glColor3f(0,0,1);
	glPushMatrix();
	glTranslatef(0,0, 0.8f);
	glutSolidCone(0.0325,0.2, 4,1);
	//Draw label
	glTranslatef(0,0.0625,0.225f);
	RenderSpacedBitmapString(0,0,0,GLUT_BITMAP_HELVETICA_10, label[Z]);
	glPopMatrix();
	glutSolidCone(0.0225,1, 4,1);
	
	glColor3f(1,0,0);
	glRotatef(90,0,1,0);
	glPushMatrix();
	glTranslatef(0,0,0.8f);
	glutSolidCone(0.0325,0.2, 4,1);
	//Draw label
	glTranslatef(0,0.0625,0.225f);
	RenderSpacedBitmapString(0,0,0,GLUT_BITMAP_HELVETICA_10, label[X]);
	glPopMatrix();
	glutSolidCone(0.0225,1, 4,1);
	
	glColor3f(0,1,0);
	glRotatef(90,-1,0,0);
	glPushMatrix();
	glTranslatef(0,0, 0.8f);
	glutSolidCone(0.0325,0.2, 4,1);
	//Draw label
	glTranslatef(0,0.0625,0.225f);
	RenderSpacedBitmapString(0,0,0,GLUT_BITMAP_HELVETICA_10, label[Y]);
	glPopMatrix();
	glutSolidCone(0.0225,1, 4,1);
	
	glPopMatrix();
}
void DrawGrid(int GRID_SIZE) {
	glBegin(GL_LINES);
	glColor3f(0.75f, 0.75f, 0.75f);
	for(int i=-GRID_SIZE;i<=GRID_SIZE;i++)
	{
		glVertex3f((float)i,0,(float)-GRID_SIZE);
		glVertex3f((float)i,0,(float)GRID_SIZE);
		
		glVertex3f((float)-GRID_SIZE,0,(float)i);
		glVertex3f((float)GRID_SIZE,0,(float)i);
	}
	glEnd();
}


void DrawActor(PxRigidActor* actor) {
	PxU32 nShapes = actor->getNbShapes();
	PxShape **shapes = new PxShape*[nShapes];
	
	actor->getShapes(shapes, nShapes);
	while (nShapes--) {
		DrawShape(shapes[nShapes], actor);
	}
	delete [] shapes;
}

void RenderActors()  {
	// Render all the actors in the scene
	for (int i = 0; i < M0.size(); i++) {
		M0[i]->glutDraw();
	}
	for (int i = 0; i < swing.size(); i++) {
		swing[i]->glutDraw();
	}
	for (int i = 0; i < Actors.size(); i++) {
		DrawActor(Actors[i]);
	}
}

void DrawShape(PxShape* shape, PxRigidActor *actor) {
	PxGeometryType::Enum type = shape->getGeometryType();
	switch(type)
	{
		case PxGeometryType::eBOX:
			glColor3f(0.5,0.5,0.5);
			DrawBox(shape, actor);
			break;
			
		case PxGeometryType::eSPHERE:
			glColor3f(0.5,0.5,0.5);
			DrawSphere(shape, actor);
			break;
			
		case PxGeometryType::eCAPSULE:
			glColor3f(0.5,0.5,0.5);
			DrawCapsule(shape, actor);
			break;
			
		default:
			cerr<<"Shape not supported."<<type<<endl;
	}
}

void DrawBox(PxShape* pShape, PxRigidActor *actor) {
	PxTransform pT = PxShapeExt::getGlobalPose(*pShape, *actor);
	PxBoxGeometry bg;
	pShape->getBoxGeometry(bg);
	PxMat33 m = PxMat33(pT.q );
	float mat[16];
	getColumnMajor(m, pT.p, mat);
	glPushMatrix();
	glMultMatrixf(mat);
	glScalef(bg.halfExtents.x * 2, bg.halfExtents.y * 2, bg.halfExtents.z * 2);
	glutSolidCube(1);
	glPopMatrix();
}
void DrawSphere(PxShape* pShape, PxRigidActor *actor) {
	PxTransform pT = PxShapeExt::getGlobalPose(*pShape, *actor);
	PxSphereGeometry bg;
	pShape->getSphereGeometry(bg);
	PxMat33 m = PxMat33(pT.q );
	float mat[16];
	getColumnMajor(m,pT.p, mat);
	glPushMatrix();
	glMultMatrixf(mat);
	glutSolidSphere(bg.radius, 20, 20);
	glPopMatrix();
}
void DrawCapsule(PxShape* pShape, PxRigidActor *actor) {
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


void Shutdown() {
	Clean();
	gScene->release();
	delete physicsEngine;
	delete gl;
}

void Clean() {
	for (int i = 0; i < Actors.size(); i++) {
		gScene->removeActor(*Actors[i]);
	}
	Actors.clear();
	
	for (int i = 0; i < M0.size(); i++) {
		M0[i]->Clear();
		delete M0[i];
	}
	M0.clear();
	
	for (int i = 0; i < swing.size(); i++) {
		swing[i]->Clear();
		delete swing[i];
	}
	swing.clear();
}

void CreateInitialActors() {
	// Create ground plane
	PxMaterial* ground = gPhysicsSDK->createMaterial(0.25, 0.1, 0.1);
	PxTransform pose = PxTransform(PxVec3(0.0f, 0.0f, 0.0f),PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));
	
	PxRigidStatic *plane = gPhysicsSDK->createRigidStatic(pose);
	if (!plane)
		cerr<<"create plane failed!"<<endl;
	
	PxShape* shape = plane->createShape(PxPlaneGeometry(), *ground);
	if (!shape)
		cerr<<"create shape failed!"<<endl;
	gScene->addActor(*plane);
	Actors.push_back(plane);
	
	// Create Swing
	double swing_height = 10.0;
	double seg_radius = 0.07;
	double seg_half_length = 0.30;
	int seg_num = 10;
	double half_width = 1.0;
	
	double mannequin_height = 6;
	double shoulder_width = half_width * 1.5;
	
	PxMaterial *body = gPhysicsSDK->createMaterial(0.7, 0.65, 0.1);
	PxMaterial *iron = gPhysicsSDK->createMaterial(0.5, 0.4, 0.65);
	PxMaterial *wood = gPhysicsSDK->createMaterial(0.5, 0.4, 0.5);
	
	PxReal body_density = 2.0f;
	PxReal iron_density = 7.8f;
	PxReal wood_density = 5.4f;
	
	
	Swing *s = new Swing(PxVec3(0, swing_height, 0), half_width, seg_num, seg_radius, seg_half_length,
					  iron_density, iron, wood_density, wood);
	swing.push_back(s);
	
	Mannequin *m = new Mannequin(mannequin_height, shoulder_width,
								 PxVec3(0, swing[0]->seat_height + 0.2, 0), body_density, body);
	M0.push_back(m);
	
	// Make Mannequin grasp the chain
	int hand_seg_num = 5;
	PxTransform arm_hand(PxVec3(-M0[0]->arm_length / 2, 0, 0));
	PxD6Joint *j;
	
	j = PxD6JointCreate(*gPhysicsSDK, M0[0]->lower_arm1, arm_hand,
						swing[0]->chain1->ArticLinks[hand_seg_num], PxTransform(PxVec3(0)));
	j->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	
	j = PxD6JointCreate(*gPhysicsSDK, M0[0]->lower_arm2, arm_hand,
						swing[0]->chain2->ArticLinks[hand_seg_num], PxTransform(PxVec3(0)));
	j->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	
	PxTransform leg_foot(PxVec3(-((M0[0]->leg_length) / 2 + 0.2), 0, 0));
	PxVec3 seat_foot((M0[0]->body_width - M0[0]->leg_width - 0.1) / 2, 0, 0);
	
	j = PxD6JointCreate(*gPhysicsSDK, M0[0]->lower_leg1, leg_foot,
						swing[0]->seat, PxTransform(seat_foot));
	j->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	
	j = PxD6JointCreate(*gPhysicsSDK, M0[0]->lower_leg2, leg_foot,
						swing[0]->seat, PxTransform(-seat_foot));
	j->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);
	j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	
}

PxQuat QuatRotate(PxVec3 axis, PxReal rot) {
	PxQuat ret;
	axis /= axis.magnitude();
	ret.w = cos(rot / 2);
	ret.x = sin(rot / 2) * axis.x;
	ret.y = sin(rot / 2) * axis.y;
	ret.z = sin(rot / 2) * axis.z;
	return ret;
}
PxVec3 VecRotate (PxReal theta, PxReal phi) {
	PxVec3 ret;
	ret.x = PxSin(theta) * PxCos(phi);
	ret.y = PxSin(theta) * PxSin(phi);
	ret.z = PxCos(theta);
	return ret;
}

PxRigidDynamic* CreateBox (PxVec3 pos, PxVec3 axis, PxReal rot, PxVec3 dimension,
						   PxReal density, PxMaterial* mMaterial) {
	PxTransform transform(pos, QuatRotate(axis, rot));
	PxBoxGeometry geometry(dimension);
	PxRigidDynamic *actor = PxCreateDynamic(*gPhysicsSDK, transform, geometry, *mMaterial, density);
	if (!actor)
		cerr<<"create actor failed!"<<endl;
	actor->setAngularDamping(0.75);
	actor->setLinearVelocity(PxVec3(0, 0, 0));
	gScene->addActor(*actor);
	return actor;
}
PxRigidDynamic* CreateSphere(PxVec3 pos, PxReal radius,
							 PxReal density, PxMaterial* mMaterial) {
	// Add a single-shape actor to the scene
	PxTransform transform(pos, PxQuat(PxIdentity));
	PxSphereGeometry geometry(radius);
	
	PxRigidDynamic *actor = PxCreateDynamic(*gPhysicsSDK, transform, geometry, *mMaterial, density);
	if (!actor)
		cerr<<"create actor failed!"<<endl;
	actor->setAngularDamping(0.75);
	actor->setLinearVelocity(PxVec3(0,0,0));
	gScene->addActor(*actor);
	return actor;
}
PxRigidDynamic* CreateCapsule(PxVec3 pos, PxVec3 axis, PxReal rot, PxReal radius, PxReal half_height,
							  PxReal density, PxMaterial *mMaterial) {
	PxTransform transform(pos, QuatRotate(axis, rot));
	PxCapsuleGeometry geometry(radius, half_height);
	PxRigidDynamic *actor = PxCreateDynamic(*gPhysicsSDK, transform, geometry, *mMaterial, density);
	if (!actor)
		cerr<<"create actor failed!"<<endl;
	actor->setAngularDamping(0.75);
	actor->setLinearVelocity(PxVec3(0, 0, 0));
	gScene->addActor(*actor);
	return actor;
}

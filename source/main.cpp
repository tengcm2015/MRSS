//
//  main.cpp
//  MRSS
//
//  Created by tengcm on 14/10/29.
//
//

#include <iostream>
#include <vector>

#include "GlobalState.hpp"
#include "PhysX.hpp"
#include "GL.hpp"
#include "objects/SwingTester.hpp"

using namespace std;

#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768

void MouseFunc(int button , int state, int x, int y);

void KeyPressFunc(unsigned char key, int x, int y);

void SpecialKeyFunc(int key, int x, int y);

void ResizeWindow(int w, int h);

void MotionFunc(int x, int y);

void IdleFunc();

void Animate();

void StepPhysX();

void SetOrthoForFont();
void ResetPerspectiveProjection();
void RenderSpacedBitmapString(int ,int ,int ,void* , char*);
void DrawAxes();
void DrawGrid(int);

void RenderActors();

void Shutdown();
void Clean();

void CreateInitialObjects();

static PxPhysics* gPhysicsSDK = nullptr;
static PxScene* gScene = nullptr;

std::unique_ptr<SwingTester> gTester = nullptr;

PxRigidStatic *planeActor = nullptr;

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

	gl = std::make_unique<GL>();
	physicsEngine = std::make_unique<PhysX>();

	gScene = physicsEngine->createPxScene();
	gPhysicsSDK = physicsEngine->getPhysics();
	CreateInitialObjects();

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
			gTester->setDriveOn(!gTester->driveIsOn());
			break;

		case 'H':
		case 'h':
			globalState.help = !globalState.help;
			break;

		case 'M':
		case 'm':
			globalState.manual = !globalState.manual;
			gTester->resetAutoState();
			break;

		case 'P':
		case 'p':
			globalState.paused = !globalState.paused;
			break;

		case 'R':
		case 'r':
			Clean();
			CreateInitialObjects();
			// global variable initializations
			globalState.countdown = 1000;
			globalState.elapsed = 0;
			globalState.start = glutGet(GLUT_ELAPSED_TIME);

			globalState.fps = 0;
			globalState.startTime = 0;
			globalState.totalFrames = 0;

			break;

		case 32:	// Space Bar
			if (gTester->getPose() == TESTER_POSE::STAND)
				gTester->setPose(TESTER_POSE::CROUCH);
			else
				gTester->setPose(TESTER_POSE::STAND);
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
	if (gTester->isLocked()) {
		if (globalState.elapsed < globalState.countdown) {
			if (globalState.paused) {
				globalState.countdown -= globalState.elapsed;
				globalState.start = glutGet(GLUT_ELAPSED_TIME);
				globalState.elapsed = 0;
			} else {
				globalState.elapsed = glutGet(GLUT_ELAPSED_TIME) - globalState.start;
			}
		} else {
			gTester->unlock();
			gTester->setPose(TESTER_POSE::STAND);
			gTester->setDriveOn(true);
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
	float currentAngle = gTester->getSwingAngle();
	if (gScene){
		if (globalState.help) {
			sprintf(buffer, "D: toggle drive\nM: manual/auto\nP: pause/resume\nR: reset\nSpace: toggle pose(manual)\nArrow Keys: move perspective\nEsc: exit\nH to return");
		} else {
			char l1[1024], l2[1024], l3[1024];

			sprintf(l1, "FPS: %3.2f Angle: %3.2f", globalState.fps, currentAngle);
//			PxTolerancesScale t = gPhysicsSDK->getTolerancesScale();
//			sprintf(l1, "%lf %lf %lf", t.length, t.mass, t.speed);

			sprintf(l2, "Drive:");
			if (gTester->driveIsOn()) {
				strcat(l2, " On  ");
			} else {
				strcat(l2, " Off  ");
			}
			if (globalState.manual) {
				strcat(l2, " Manual ");
			} else {
				strcat(l2, " Auto ");
			}
			if (gTester->getPose() == TESTER_POSE::STAND) {
				strcat(l2, " Stand");
			} else {
				strcat(l2, " Crouch");
			}
			if (globalState.paused) {
				strcat(l2, " Pause");
			} else {
				if (gTester->isLocked()) {
					strcat(l2, " Initializing...");
				}
				StepPhysX();
			}

			sprintf(l3, "H for help");

			sprintf(buffer, "%s\n%s\n%s", l1, l2, l3);
		}
	}

	// Automatic Mannequin
	if (!globalState.manual) {
		gTester->autoPose();
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

void RenderActors()  {
	// Render all the actors in the scene
	gTester->draw();
}

void Shutdown() {
	gScene->release();
}

void Clean() {
	gTester->removeFromPxScene(gScene);
	gScene->removeActor(*planeActor);
	planeActor->release();
}

void CreateInitialObjects() {
	// Create ground plane
	PxMaterial* ground = gPhysicsSDK->createMaterial(0.25, 0.1, 0.1);
	PxTransform pose = PxTransform(PxVec3(0.0f, 0.0f, 0.0f),PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));

	planeActor = gPhysicsSDK->createRigidStatic(pose);
	if (!planeActor)
		cerr<<"create plane failed!"<<endl;

	PxShape* shape = planeActor->createShape(PxPlaneGeometry(), *ground);
	if (!shape)
		cerr<<"create shape failed!"<<endl;
	gScene->addActor(*planeActor);

	gTester = make_unique<SwingTester>(0, 0);
	gTester->addToPxScene(gScene);
}

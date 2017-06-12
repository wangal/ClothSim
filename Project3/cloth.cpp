/*

  USC/Viterbi/Computer Science
  "cloth Cube" Assignment 1 starter code

  Your name:
  Alex Wang

*/

#include <string>
#include <ctime>

#include "cloth.h"
#include "input.h"
#include "physics.h"


// camera parameters
double Theta = pi / 6;
double Phi = pi / 6;
double R = 6;

// mouse control
int g_iMenuId;
int g_vMousePos[2];
int g_iLeftMouseButton,g_iMiddleMouseButton,g_iRightMouseButton;

// number of images saved to disk so far
int sprite=0;

// these variables control what is displayed on screen
int shear=0, bend=0, structural=1, pause=0, viewingMode=0, saveScreenToFile=0;

struct world cloth;

int windowWidth, windowHeight;

void myinit()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(90.0,1.0,0.01,1000.0);

  // set background color to grey
  glClearColor(0.0, 0.0, 0.0, 0.0);

  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_POLYGON_SMOOTH);
  glEnable(GL_LINE_SMOOTH);

  return; 
}

void reshape(int w, int h) 
{
  // Prevent a divide by zero, when h is zero.
  // You can't make a window of zero height.
  if(h == 0)
    h = 1;

  glViewport(0, 0, w, h);

  // Reset the coordinate system before modifying
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // Set the perspective
  double aspectRatio = 1.0 * w / h;
  gluPerspective(60.0f, aspectRatio, 0.01f, 1000.0f);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity(); 

  windowWidth = w;
  windowHeight = h;

  glutPostRedisplay();
}

int pointMap(int side, int i, int j)
{
	int r;

	switch (side)
	{
	case 1: //[i][j][0] bottom face
		r = 64 * i + 8 * j;
		break;
	case 6: //[i][j][7] top face
		r = 64 * i + 8 * j + 7;
		break;
	case 2: //[i][0][j] front face
		r = 64 * i + j;
		break;
	case 5: //[i][7][j] back face
		r = 64 * i + 56 + j;
		break;
	case 3: //[0][i][j] left face
		r = 8 * i + j;
		break;
	case 4: //[7][i][j] right face
		r = 448 + 8 * i + j;
		break;
	}

	return r;
}

void processNeighbor(struct world * cloth, int i, int j, int ip, int jp)
{
	int n = cloth->size;
	if (i >= 0 && i < n && j >= 0 && j < n)
	{
		if ((i + ip) >= 0 && (i + ip) < n && (j + jp) >= 0 && (j + jp) < n)
		{
			glVertex3f(cloth->p[i][j].x, cloth->p[i][j].y, cloth->p[i][j].z);
			glVertex3f(cloth->p[i + ip][j + jp].x, cloth->p[i + ip][j + jp].y, cloth->p[i + ip][j + jp].z);
		}
	}
}

void showCube(struct world * cloth)
{
	int i, j, k, ip, jp, kp;
	point r1, r2, r3, r4; // aux variables

						  /* normals buffer and counter for Gourad shading*/
	struct point normal[32][32];
	int counter[32][32];

	int face;
	double faceFactor, length;

	if (fabs(cloth->p[0][0].x) > 10)
	{
		printf("Your cube somehow escaped way out of the box.\n");
		exit(0);
	}

	if (viewingMode == 1) // render wireframe
	{
		glLineWidth(1);
		glPointSize(2);
		glDisable(GL_LIGHTING);
		for (i = 0; i < cloth->size; i++)
			for (j = 0; j < cloth->size; j++)
			{
				glBegin(GL_POINTS); // draw point
				glColor4f(0, 0, 0, 0);
				glVertex3f(cloth->p[i][j].x, cloth->p[i][j].y, cloth->p[i][j].z);
				glEnd();

				glBegin(GL_LINES);

				// structural
				if (structural == 1)
				{
					glColor4f(0, 0, 1, 1);
					processNeighbor(cloth, i, j, 1, 0);
					processNeighbor(cloth, i, j, 0, 1);
				}

				// shear
				if (shear == 1)
				{
					glColor4f(0, 1, 0, 1);
					processNeighbor(cloth, i, j, 1, 1);
					processNeighbor(cloth, i, j, 1, -1);
				}

				// bend
				if (bend == 1)
				{
					glColor4f(1, 0, 0, 1);
					processNeighbor(cloth, i, j, 2, 0);
					processNeighbor(cloth, i, j, 0, 2);
				}

				glEnd();
			}
		glEnable(GL_LIGHTING);
	}

	else
	{
		for (int face = 0; face < 2; ++face)
		{
			for (i = 0; i < cloth->size; i++) // reset buffers
				for (j = 0; j < cloth->size; j++)
				{
					normal[i][j].x = 0; normal[i][j].y = 0; normal[i][j].z = 0;
					counter[i][j] = 0;
				}

			/* process triangles, accumulate normals for Gourad shading */

			for (i = 0; i < cloth->size - 1; i++)
				for (j = 0; j < cloth->size - 1; j++) // process block (i,j)
				{
					if (face  > 0)
						glFrontFace(GL_CCW); // the usual definition of front face
					else
						glFrontFace(GL_CW); // flip definition of orientation

					pDIFFERENCE(cloth->p[i + 1][j], cloth->p[i][j], r1); // first triangle
					pDIFFERENCE(cloth->p[i][j + 1], cloth->p[i][j], r2);
					CROSSPRODUCTp(r1, r2, r3);
					pNORMALIZE(r3);
					pSUM(normal[i + 1][j], r3, normal[i + 1][j]);
					counter[i + 1][j]++;
					pSUM(normal[i][j + 1], r3, normal[i][j + 1]);
					counter[i][j + 1]++;
					pSUM(normal[i][j], r3, normal[i][j]);
					counter[i][j]++;

					pDIFFERENCE(cloth->p[i][j + 1], cloth->p[i + 1][j + 1], r1); // second triangle
					pDIFFERENCE(cloth->p[i + 1][j], cloth->p[i + 1][j + 1], r2);
					CROSSPRODUCTp(r1, r2, r3);
					pNORMALIZE(r3);
					pSUM(normal[i + 1][j], r3, normal[i + 1][j]);
					counter[i + 1][j]++;
					pSUM(normal[i][j + 1], r3, normal[i][j + 1]);
					counter[i][j + 1]++;
					pSUM(normal[i + 1][j + 1], r3, normal[i + 1][j + 1]);
					counter[i + 1][j + 1]++;
				}

			/* the actual rendering */
			for (j = 1; j < cloth->size; j++)
			{
				glBegin(GL_TRIANGLE_STRIP);
				for (i = 0; i < cloth->size; i++)
				{
					glNormal3f(normal[i][j].x / counter[i][j], normal[i][j].y / counter[i][j],
						normal[i][j].z / counter[i][j]);
					glVertex3f(cloth->p[i][j].x, cloth->p[i][j].y, cloth->p[i][j].z);
					glNormal3f(normal[i][j - 1].x / counter[i][j - 1], normal[i][j - 1].y / counter[i][j - 1],
						normal[i][j - 1].z / counter[i][j - 1]);
					glVertex3f(cloth->p[i][j - 1].x, cloth->p[i][j - 1].y, cloth->p[i][j - 1].z);
				}
				glEnd();
			}
		}
	}
	// forms the ground
	
	glBegin(GL_TRIANGLE_STRIP);
	glNormal3f(0, 1, 0);
	glVertex3f(-2, -2, -2);

	glNormal3f(0, 1, 0);
	glVertex3f(2, -2, -2);

	glNormal3f(0, 1, 0);
	glVertex3f(-2, 2, -2);

	glNormal3f(0, 1, 0);
	glVertex3f(2, 2, -2);
	glEnd();

	// form cubes
	for (int i = 0; i < cloth->cubes.size(); ++i)
	{
		point p = cloth->cubes[i].p;
		float w = cloth->cubes[i].width / 2.0;
		float h = cloth->cubes[i].height / 2.0;
		float l = cloth->cubes[i].length / 2.0;
		glPushMatrix();
		glTranslatef(p.x, p.y, p.z);

		// bottom
		glBegin(GL_QUADS);
		glNormal3f(0, 0, -1);
		glVertex3f(l, -w, -h);
		glVertex3f(-l, -w, -h);
		glVertex3f(-l, w, -h);
		glVertex3f(l, w, -h);
		glEnd();

		// top
		glBegin(GL_QUADS);
		glNormal3f(0, 0, 1);
		glVertex3f(l, w, h);
		glVertex3f(-l, w, h);
		glVertex3f(-l, -w, h);
		glVertex3f(l, -w, h);
		glEnd();

		// front
		glBegin(GL_QUADS);
		glNormal3f(1, 0, 0);
		glVertex3f(l, -w, h);
		glVertex3f(l, -w, -h);
		glVertex3f(l, w, -h);
		glVertex3f(l, w, h);
		glEnd();

		// back
		glBegin(GL_QUADS);
		glNormal3f(-1, 0, 0);
		glVertex3f(-l, w, h);
		glVertex3f(-l, w, -h);
		glVertex3f(-l, -w, -h);
		glVertex3f(-l, -w, h);
		glEnd();

		// left
		glBegin(GL_QUADS);
		glNormal3f(0, -1, 0);
		glVertex3f(-l, -w, h);
		glVertex3f(-l, -w, -h);
		glVertex3f(l, -w, -h);
		glVertex3f(l, -w, h);
		glEnd();

		// right
		glBegin(GL_QUADS);
		glNormal3f(0, 1, 0);
		glVertex3f(l, w, h);
		glVertex3f(l, w, -h);
		glVertex3f(-l, w, -h);
		glVertex3f(-l, w, h);
		glEnd();


		glPopMatrix();
	}

	// form spheres
	for (int i = 0; i < cloth->spheres.size(); ++i)
	{
		point p = cloth->spheres[i].p;
		glPushMatrix();
		glTranslatef(p.x, p.y, p.z);
		glutSolidSphere(cloth->spheres[i].radius, 50, 50);
		glPopMatrix();
	}
}

void display()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // camera parameters are Phi, Theta, R
  gluLookAt(R * cos(Phi) * cos (Theta), R * sin(Phi) * cos (Theta), R * sin (Theta),
	        0.0,0.0,0.0, 0.0,0.0,1.0);


  /* Lighting */
  /* You are encouraged to change lighting parameters or make improvements/modifications
     to the lighting model . 
     This way, you will personalize your assignment and your assignment will stick out. 
  */

  // global ambient light
  GLfloat aGa[] = { 0.0, 0.0, 0.0, 0.0 };
  
  // light 's ambient, diffuse, specular
  GLfloat lKa0[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd0[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat lKs0[] = { 1.0, 1.0, 1.0, 1.0 };

  GLfloat lKa1[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd1[] = { 1.0, 0.0, 0.0, 1.0 };
  GLfloat lKs1[] = { 1.0, 0.0, 0.0, 1.0 };

  GLfloat lKa2[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd2[] = { 1.0, 1.0, 0.0, 1.0 };
  GLfloat lKs2[] = { 1.0, 1.0, 0.0, 1.0 };

  GLfloat lKa3[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd3[] = { 0.0, 1.0, 1.0, 1.0 };
  GLfloat lKs3[] = { 0.0, 1.0, 1.0, 1.0 };

  GLfloat lKa4[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd4[] = { 0.0, 0.0, 1.0, 1.0 };
  GLfloat lKs4[] = { 0.0, 0.0, 1.0, 1.0 };

  GLfloat lKa5[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd5[] = { 1.0, 0.0, 1.0, 1.0 };
  GLfloat lKs5[] = { 1.0, 0.0, 1.0, 1.0 };

  GLfloat lKa6[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd6[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat lKs6[] = { 1.0, 1.0, 1.0, 1.0 };

  GLfloat lKa7[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd7[] = { 0.0, 1.0, 1.0, 1.0 };
  GLfloat lKs7[] = { 0.0, 1.0, 1.0, 1.0 };

  // light positions and directions
  GLfloat lP0[] = { -1.999, -1.999, -1.999, 1.0 };
  GLfloat lP1[] = { 1.999, -1.999, -1.999, 1.0 };
  GLfloat lP2[] = { 1.999, 1.999, -1.999, 1.0 };
  GLfloat lP3[] = { -1.999, 1.999, -1.999, 1.0 };
  GLfloat lP4[] = { -1.999, -1.999, 1.999, 1.0 };
  GLfloat lP5[] = { 1.999, -1.999, 1.999, 1.0 };
  GLfloat lP6[] = { 1.999, 1.999, 1.999, 1.0 };
  GLfloat lP7[] = { -1.999, 1.999, 1.999, 1.0 };
  
  // jelly material color

  GLfloat mKa[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat mKd[] = { 0.3, 0.3, 0.3, 1.0 };
  GLfloat mKs[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat mKe[] = { 0.0, 0.0, 0.0, 1.0 };

  /* set up lighting */
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, aGa);
  glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

  // set up cube color
  glMaterialfv(GL_FRONT, GL_AMBIENT, mKa);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, mKd);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mKs);
  glMaterialfv(GL_FRONT, GL_EMISSION, mKe);
  glMaterialf(GL_FRONT, GL_SHININESS, 120);
    
  // macro to set up light i
  #define LIGHTSETUP(i)\
  glLightfv(GL_LIGHT##i, GL_POSITION, lP##i);\
  glLightfv(GL_LIGHT##i, GL_AMBIENT, lKa##i);\
  glLightfv(GL_LIGHT##i, GL_DIFFUSE, lKd##i);\
  glLightfv(GL_LIGHT##i, GL_SPECULAR, lKs##i);\
  glEnable(GL_LIGHT##i)
  
  LIGHTSETUP (0);
  LIGHTSETUP (1);
  LIGHTSETUP (2);
  LIGHTSETUP (3);
  LIGHTSETUP (4);
  LIGHTSETUP (5);
  LIGHTSETUP (6);
  LIGHTSETUP (7);

  // enable lighting
  glEnable(GL_LIGHTING);    
  glEnable(GL_DEPTH_TEST);

  // show the cube
  showCube(&cloth);

  glDisable(GL_LIGHTING);

  // show the bounding box
//  showBoundingBox();
 
  glutSwapBuffers();
}

// finds the overall spring force between a point and its neighbor
point computeHookDampForce(struct world *cloth, int i, int j, intPoint p)
{
	// setup variables
	point hookForce;
	point dampForce;

	point L;
	point L_unit;
	double L_magnitude;

	point diff;

	// find difference from newPoint to basePoint: A <- B
	pDIFFERENCE(cloth->p[i][j], cloth->p[p.x][p.y], L);
	L_magnitude = L.magnitude();

	// get unit vector for L
	pMULTIPLY(L, 1 / L_magnitude, L_unit);

	// Hook's law: F = -kh * (|L| - R) * (L / |L|)
	double hookValue = -cloth->kElastic * (L_magnitude - p.restLength);
	pMULTIPLY(L_unit, hookValue, hookForce);

	// Damping: F = -kd * (((va - vb) * L) / |L|) * (L / |L|)
	pDIFFERENCE(cloth->v[i][j], cloth->v[p.x][p.y], diff);
	double dampValue = -cloth->dElastic * diff.dotProduct(L) / L_magnitude;
	pMULTIPLY(L_unit, dampValue, dampForce);

	// calculates overall spring force
	point force;
	pSUM(hookForce, dampForce, force);

	return force;
}

// finds the spring force using collision
point computeHookDampCollisionForce(struct world *cloth, int i, int j, point collisionPoint)
{
	// 1) magnitude from penetration point to contact point
	// 2) direction normal to surface
	point hookForce;
	point dampForce;

	point L;
	point L_unit;
	double L_magnitude;

	// find difference from newPoint to basePoint: A <- B
	// NOTE: difference is the normal of the plane
	// NOTE: magnitude is the from point to collision point
	pDIFFERENCE(cloth->p[i][j], collisionPoint, L);
	L_magnitude = L.magnitude();

	// get unit vector for L
	pMULTIPLY(L, 1 / L_magnitude, L_unit);

	// Hook's law: F = -kh * (|L| - R) * (L / |L|)
	// NOTE: R is zero for collision
	double hookValue = -cloth->kCollision * (L_magnitude);
	pMULTIPLY(L_unit, hookValue, hookForce);

	// Damping: F = -kd * (((va - vb) * L) / |L|) * (L / |L|)
	// NOTE: velocity at b is zero
	double dampValue = -cloth->dCollision * cloth->v[i][j].dotProduct(L) / L_magnitude;
	pMULTIPLY(L_unit, dampValue, dampForce);

	// calculates overall spring force
	point force;
	pSUM(hookForce, dampForce, force);

	return force;
}

// finds the spring force using collision
point computeHookDampCollisionForce(struct world *cloth, point p, point v, point collisionPoint)
{
	// 1) magnitude from penetration point to contact point
	// 2) direction normal to surface
	point hookForce;
	point dampForce;

	point L;
	point L_unit;
	double L_magnitude;

	// find difference from newPoint to basePoint: A <- B
	// NOTE: difference is the normal of the plane
	// NOTE: magnitude is the from point to collision point
	pDIFFERENCE(p, collisionPoint, L);
	L_magnitude = L.magnitude();

	// get unit vector for L
	pMULTIPLY(L, 1 / L_magnitude, L_unit);

	// Hook's law: F = -kh * (|L| - R) * (L / |L|)
	// NOTE: R is zero for collision
	double hookValue = -cloth->kCollision * (L_magnitude);
	pMULTIPLY(L_unit, hookValue, hookForce);

	// Damping: F = -kd * (((va - vb) * L) / |L|) * (L / |L|)
	// NOTE: velocity at b is zero
	double dampValue = -cloth->dCollision * v.dotProduct(L) / L_magnitude;
	pMULTIPLY(L_unit, dampValue, dampForce);

	// calculates overall spring force
	point force;
	pSUM(hookForce, dampForce, force);

	return force;
}

void computeAcceleration(struct world *cloth, struct point a[32][32])
{
	struct point basePoint;

	// cycle through every point of the cloth
	for (int i = 0; i < cloth->size; ++i)
	{
		for (int j = 0; j < cloth->size; ++j)
		{
			// assign zero acceleration
			a[i][j].x = 0;
			a[i][j].y = 0;
			a[i][j].z = 0;

			// base point for checking acceleration
			basePoint = cloth->p[i][j];

			// grab the list of the base point's neighbors
			vector<intPoint> neighbors = cloth->neighbors[i][j];

			// cycle through all the neighbors for structural, shear, and bend
			for (int n = 0; n < neighbors.size(); ++n)
			{
				intPoint p = neighbors[n];

				// stores spring force in a
				if ((p.x == 0 && p.y == 0) || (p.x == 0 && p.y == cloth->size - 1))
				{
					pSUM(a[i][j], computeHookDampForce(cloth, i, j, p), a[i][j]);
					pSUM(a[i][j], computeHookDampForce(cloth, i, j, p), a[i][j]);
				}
				else
				{
					pSUM(a[i][j], computeHookDampForce(cloth, i, j, p), a[i][j]);
				}
			}
		}
	}

	// Collision Detection
	// penalty force
	for (int i = 0; i < cloth->size; ++i)
	{
		for (int j = 0; j < cloth->size; ++j)
		{
			point basePoint = cloth->p[i][j];
			point sub;

			// collision against the floor
			if (basePoint.z < -1.95)
			{
				sub = basePoint;
				sub.z = -1.95;
				pSUM(a[i][j], computeHookDampCollisionForce(cloth, i, j, sub), a[i][j]);
			}
		}
	}

	point r1;
	point r2;
	point r3;
	point normal;
	double area;
	point airForce;
	airForce.x = 0;
	airForce.y = 0;
	airForce.z = 0;

	// cycle through all the triangles
	for (int i = 0; i < cloth->size - 1; i++)
	{
		for (int j = 0; j < cloth->size - 1; j++) // process block (i,j)
		{
			// find velocity of first triangle
			point v;
			double avg = 1.0 / 3.0;
			pSUM(cloth->v[i + 1][j], cloth->v[i][j], v);
			pSUM(v, cloth->p[i][j + 1], v);
			pMULTIPLY(v, avg, v);

			if (v.magnitude() > 0.0001)
			{
				// find the normal of triangle
				pDIFFERENCE(cloth->p[i + 1][j], cloth->p[i][j], r1); // first triangle
				pDIFFERENCE(cloth->p[i][j + 1], cloth->p[i][j], r2);
				CROSSPRODUCTp(r1, r2, r3);
				pMULTIPLY(r3, 1 / r3.magnitude(), normal);

				// find the area of triangle
				area = 0.5 * r3.magnitude();
				area = v.dotProduct(normal) / v.magnitude();

				// find the air resistance force
				double value = -0.5  * cloth->airDensity * pow(v.magnitude(), 2) * cloth->dragK * area;
				pMULTIPLY(normal, value, airForce);

				// distribute force amongst points of triangle
				pMULTIPLY(airForce, (1.0 / 3.0), airForce);
				pSUM(airForce, a[i][j], a[i][j]);
				pSUM(airForce, a[i + 1][j], a[i + 1][j]);
				pSUM(airForce, a[i][j + 1], a[i][j + 1]);
			}

			// find velocity of second triangle
			avg = 1.0 / 3.0;
			pSUM(cloth->v[i][j + 1], cloth->v[i + 1][j], v);
			pSUM(v, cloth->p[i + 1][j + 1], v);
			pMULTIPLY(v, avg, v);

			if (v.magnitude() > 0.0001)
			{
				// find the normal of triangle
				pDIFFERENCE(cloth->p[i][j + 1], cloth->p[i + 1][j + 1], r1); // second triangle
				pDIFFERENCE(cloth->p[i + 1][j], cloth->p[i + 1][j + 1], r2);
				CROSSPRODUCTp(r1, r2, r3);
				pMULTIPLY(r3, 1 / r3.magnitude(), normal);

				// find the area of triangle
				area = 0.5 * r3.magnitude();
				area = v.dotProduct(normal) / v.magnitude();

				// find the air resistance force
				double value = -0.5  * cloth->airDensity * pow(v.magnitude(), 2) * cloth->dragK * area;
				pMULTIPLY(normal, value, airForce);

				// distribute force amongst points of triangle
				pMULTIPLY(airForce, (1.0 / 3.0), airForce);
				pSUM(airForce, a[i + 1][j], a[i + 1][j]);
				pSUM(airForce, a[i][j + 1], a[i][j + 1]);
				pSUM(airForce, a[i + 1][j + 1], a[i + 1][j + 1]);
			}
		}
	}

	// compute actual acceleration via a = F/m
	for (int i = 0; i < cloth->size; ++i)
	{
		for (int j = 0; j < cloth->size; ++j)
		{
			pMULTIPLY(a[i][j], (1 / cloth->mass), a[i][j]);
		}
	}

	point gravity;
	gravity.x = 0;
	gravity.y = 0;
	gravity.z = cloth->gravity;

	// add gravity force
	for (int i = 0; i < cloth->size; ++i)
	{
		for (int j = 0; j < cloth->size; ++j)
		{
			pSUM(a[i][j], gravity, a[i][j]);
		}
	}

	point a_cloth[32][32];
	
	// determine if the point intersection with another in dt time
	for (int i = 0; i < cloth->size; ++i)
	{
		for (int j = 0; j < cloth->size; ++j)
		{
			a_cloth[i][j].x = 0;
			a_cloth[i][j].y = 0;
			a_cloth[i][j].z = 0;

			point basePoint;
			point sub;
			point v;

			// gets the future position within dt
			pMULTIPLY(a[i][j], cloth->dt, sub);
			pSUM(sub, cloth->v[i][j], v);
			pMULTIPLY(v, cloth->dt, sub);
			pSUM(sub, cloth->p[i][j], basePoint);

			// collision against itself
			for (int k = 0; k < cloth->size; ++k)
			{
				for (int l = 0; l < cloth->size; ++l)
				{
					if (((i - k) * (i - k) > 4) && ((j - l) * (j - l) > 4)) // ignore the neighbors within 2 ring
					{
						point basePoint2;

						pMULTIPLY(a[k][l], cloth->dt, sub);
						pSUM(sub, cloth->v[k][l], sub);
						pMULTIPLY(sub, cloth->dt, sub);
						pSUM(sub, cloth->p[k][l], basePoint2);

						point diff;
						pDIFFERENCE(basePoint, basePoint2, diff);

						// treat cloth to cloth as 2 spheres
						if (diff.magnitude() < 2 * cloth->pointTolerance)
						{
							// repel the points
							double value = 2 * cloth->pointTolerance - diff.magnitude();

							// half the value since this will be done also on other point later on
							value /= 2.0;
							pMULTIPLY(diff, 1/ diff.magnitude(), diff);
							pMULTIPLY(diff, value, sub);
							pSUM(sub, basePoint, sub);

							a_cloth[i][j] = computeHookDampCollisionForce(cloth, basePoint, v, sub);
						}
					}
				}
			}

			// collision against spheres
			for (int k = 0; k < cloth->spheres.size(); ++k)
			{
				sphere s = cloth->spheres[k];
				point diff;
				pDIFFERENCE(basePoint, s.p, diff);

				// offset by distance between points to prevent clipping into sphere
				double radius = s.radius + cloth->length;

				if (diff.magnitude() < radius) // point is in the sphere -> determine collision point
				{
					point collision;
					float total = 1 / diff.magnitude();

					// normalized vector
					pMULTIPLY(diff, total, diff);

					// get collision point
					pMULTIPLY(diff, radius, diff);
					pSUM(s.p, diff, sub);
					/*
					printf("%lf, %lf, %lf\n", basePoint.x, basePoint.y, basePoint.z);
					printf("%lf, %lf, %lf\n", sub.x, sub.y, sub.z);
					printf("%lf\n", diff.magnitude());
					*/
					pSUM(a_cloth[i][j], computeHookDampCollisionForce(cloth, basePoint, v, sub), a_cloth[i][j]);
				}
			}
		}
	}

	// offset acceleration such that cloth to cloth collision does not occur
	for (int i = 0; i < cloth->size; ++i)
	{
		for (int j = 0; j < cloth->size; ++j)
		{
			pMULTIPLY(a_cloth[i][j], (1 / cloth->mass), a_cloth[i][j]);
			pSUM(a[i][j], a_cloth[i][j], a[i][j]);
		}
	}
	
}

void Euler(struct world * cloth)
{
	// explicit integator
	int i, j, k;
	point a[32][32];
	for (i = 0; i < cloth->size; i++)
	{
		for (j = 0; j < cloth->size; j++)
		{
			a[i][j].x = 0;
			a[i][j].y = 0;
			a[i][j].z = 0;
		}
	}

	computeAcceleration(cloth, a);

	for (i = 0; i < cloth->size; i++)
	{
		for (j = 0; j < cloth->size; j++)
		{
			if ((i == 0 && j == 0) || (i == 0 && j == cloth->size - 1))
			{
				a[i][j].x = 0;
				a[i][j].y = 0;
				a[i][j].z = 0;
			}

			// alter to dv/dt
			cloth->v[i][j].x += cloth->dt * a[i][j].x;
			cloth->v[i][j].y += cloth->dt * a[i][j].y;
			cloth->v[i][j].z += cloth->dt * a[i][j].z;

			cloth->p[i][j].x += cloth->dt * cloth->v[i][j].x;
			cloth->p[i][j].y += cloth->dt * cloth->v[i][j].y;
			cloth->p[i][j].z += cloth->dt * cloth->v[i][j].z;
		}
	}
}

// MAIN CODE
void simulate() {
	// MAIN CODE
	Euler(&cloth);
}

void doIdle()
{
	// Contains the simulation code
	simulate();

	if (GLUT_LEFT_BUTTON)
		printf("HELLO!");
  char s[20]="picxxxx.ppm";
  int i;
  
  // save screen to file
  s[3] = 48 + (sprite / 1000);
  s[4] = 48 + (sprite % 1000) / 100;
  s[5] = 48 + (sprite % 100 ) / 10;
  s[6] = 48 + sprite % 10;

  if (saveScreenToFile==1)
  {
    saveScreenshot(windowWidth, windowHeight, s);
//    saveScreenToFile=0; // save only once, change this if you want continuos image generation (i.e. animation)
    sprite++;
  }

  if (sprite >= 800) // allow only 800 snapshots
  {
    exit(0);	
  }

  if (pause == 0)
  {
    // insert code which appropriately performs one step of the cube simulation:
  }

  glutPostRedisplay();
}

intPoint createPoint(int x, int y, double diff)
{
	intPoint p;
	p.x = x; p.y = y;
	double xLength = double(x) * diff;
	double yLength = double(y) * diff;
	p.restLength = sqrt(xLength * xLength + yLength * yLength);
	return p;
}

void addObjects(world *cloth)
{
	point p;
	p.x = -0.4;
	p.y = -0.35;
	p.z = -1.5;

	// adds sphere objects
	struct sphere s;
	s.radius = 0.25;
	s.p = p;

	cloth->spheres.push_back(s);	
}

void setupCloth(world *cloth, char * fileName)
{
	FILE * file;

	file = fopen(fileName, "r");
	if (file == NULL) {
		printf("can't open file\n");
		exit(1);
	}

	double distance;

	/* read timestep size and length of cloth */
	fscanf(file, "%lf %lf\n", &cloth->dt, &distance);

	/* read physical parameters */
	fscanf(file, "%lf %lf %lf %lf\n",
		&cloth->kElastic, &cloth->dElastic, &cloth->kCollision, &cloth->dCollision);

	/* drag K and air density */
	fscanf(file, "%lf %lf\n", &cloth->dragK, &cloth->airDensity);

	/* read mass of each of the points */
	fscanf(file, "%lf\n", &cloth->mass);

	// set gravity
	fscanf(file, "%lf\n", &cloth->gravity);

	/* set physical parameters */
	int size = 32;
	double length = distance / (double)(size - 1);

	cloth->pointTolerance = length * 1.5;
	cloth->size = size;
	cloth->length = length; // length between adjacent points

	point p;
	p.x = 0;
	p.y = 0;
	p.z = 0;

	/* set initial point positions */
	p.z = -1;

	for (int i = 0; i < size; ++i)
	{
		p.x = (i * cloth->length) - 1.0;
		for (int j = 0; j < size; ++j)
		{
			p.y = (j * cloth->length) - 1.0;
			cloth->p[i][j] = p;
		}
	}

	p.x = 0;
	p.y = 0;
	p.z = 0;

	/* set initial point velocities */
	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < size; j++)
		{
			cloth->v[i][j] = p;
		}
	}

	// list for all possible neighbors based on springs attachment
	vector<intPoint> n;

	// Structural springs: point's neighbors (top, bottom, left, right)
	n.push_back(createPoint(0, 1, cloth->length));
	n.push_back(createPoint(0, -1, cloth->length));
	n.push_back(createPoint(1, 0, cloth->length));
	n.push_back(createPoint(-1, 0, cloth->length));

	// Shear springs: point's diagonal neighbors
	n.push_back(createPoint(-1, 1, cloth->length));
	n.push_back(createPoint(-1, -1, cloth->length));
	n.push_back(createPoint(1, 1, cloth->length));
	n.push_back(createPoint(1, -1, cloth->length));

	// Bend springs: point's SECOND ring neighbors
	n.push_back(createPoint(0, 2, cloth->length));
	n.push_back(createPoint(0, -2, cloth->length));
	n.push_back(createPoint(2, 0, cloth->length));
	n.push_back(createPoint(-2, 0, cloth->length));
	n.push_back(createPoint(2, 2, cloth->length));
	n.push_back(createPoint(2, -2, cloth->length));
	n.push_back(createPoint(-2, 2, cloth->length));
	n.push_back(createPoint(-2, -2, cloth->length));

	n.push_back(createPoint(2, 1, cloth->length));
	n.push_back(createPoint(2, -1, cloth->length));
	n.push_back(createPoint(-2, 1, cloth->length));
	n.push_back(createPoint(-2, -1, cloth->length));
	n.push_back(createPoint(1, 2, cloth->length));
	n.push_back(createPoint(1, -2, cloth->length));
	n.push_back(createPoint(-1, 2, cloth->length));
	n.push_back(createPoint(-1, -2, cloth->length));

	
	// add neighbors for springs
	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < size; j++)
		{
			// tracks all possible neighbors
			vector<intPoint> springs;

			// check for every possible neighbors
			for (int num = 0; num < n.size(); num++)
			{
				// finds point
				intPoint p = n[num];
				p.x += i;
				p.y += j;

				// check if the point is outside boundary or not
				if (p.x >= 0 && p.y >= 0 && p.x < size && p.y < size)
				{
					// insert the neighbor
					springs.push_back(p);
				}
			}

			// set neighbors
			cloth->neighbors[i][j] = springs;
		}
	}

	// add spheres
	addObjects(cloth);
}

int main (int argc, char ** argv)
{
  if (argc<2)
  {  
    printf ("Oops! You didn't say the cloth world file!\n");
    printf ("Usage: %s [worldfile]\n", argv[0]);
    exit(0);
  }

  setupCloth(&cloth, argv[1]);

  glutInit(&argc,argv);
  
  /* double buffered window, use depth testing, 640x480 */
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  
  windowWidth = 640;
  windowHeight = 480;
  glutInitWindowSize (windowWidth, windowHeight);
  glutInitWindowPosition (0,0);
  glutCreateWindow ("cloth cube");

  /* tells glut to use a particular display function to redraw */
  glutDisplayFunc(display);

  /* replace with any animate code */
  glutIdleFunc(doIdle);

  /* callback for mouse drags */
  glutMotionFunc(mouseMotionDrag);

  /* callback for window size changes */
  glutReshapeFunc(reshape);

  /* callback for mouse movement */
  glutPassiveMotionFunc(mouseMotion);

  /* callback for mouse button changes */
  glutMouseFunc(mouseButton);

  /* register for keyboard events */
  glutKeyboardFunc(keyboardFunc);

  /* do initialization */
  myinit();

  /* forever sink in the black hole */
  glutMainLoop();

  return(0);
}


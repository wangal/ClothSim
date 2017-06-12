/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#ifndef _CLOTH_H_
#define _CLOTH_H_

#ifdef WIN32
  #include <windows.h>
#endif

#include <vector>

using namespace std;

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "openGL-headers.h"
#include "pic.h"

#define pi 3.141592653589793238462643383279 

// camera angles
extern double Theta;
extern double Phi;
extern double R;

// number of images saved to disk so far
extern int sprite;

// mouse control
extern int g_vMousePos[2];
extern int g_iLeftMouseButton,g_iMiddleMouseButton,g_iRightMouseButton;

struct point 
{
   double x;
   double y;
   double z;

   // find the magnitude of vector
   double magnitude() {
	   return sqrt(x * x + y * y + z * z);
   }

   // dot product
   double dotProduct(point other) {
	   return x * other.x + y * other.y + z * other.z;
   }

};

// stores the information and methods for the plane
struct plane
{
	double a;
	double b;
	double c;
	double d;
	void set(double a2, double b2, double c2, double d2) {
		a = a2;
		b = b2;
		c = c2;
		d = d2;
	}

	double findValue(double x, double y, double z) {
		return a * x + b * y + c * z + d;
	}
};

// struct for storing the xyz-coordinates of spring neighbors for a point
struct intPoint
{
	int x;
	int y;
	double restLength;
};

// these variables control what is displayed on the screen
extern int shear, bend, structural, pause, viewingMode, saveScreenToFile;

struct sphere
{
	point p;
	double radius;
};

struct ccube
{
	point p;
	double length;
	double width;
	double height;
};


struct world
{
  double dt; // timestep, e.g.. 0.001
  int n; // display only every nth timepoint
  int size;
  double kElastic; // Hook's elasticity coefficient for all springs except collision springs
  double dElastic; // Damping coefficient for all springs except collision springs
  double kCollision; // Hook's elasticity coefficient for collision springs
  double dCollision; // Damping coefficient collision springs
  double mass; // mass of each of the 512 control points, mass assumed to be equal for every control point
  double pointTolerance; // radius of each point for cloth to cloth collision
  struct point * forceField; // pointer to the array of values of the force field
  struct point p[32][32]; // position of the 512 control points
  struct point v[32][32]; // velocities of the 512 control points

  // stores the spring neighbors for a point
  vector<struct intPoint> neighbors[32][32];

  // colliders for cloth interaction
  vector<struct ccube> cubes;
  vector<struct sphere> spheres;

  double dragK;
  double airDensity;
  double length;

  double gravity;

};

extern struct world cloth;

// computes crossproduct of three vectors, which are given as points
// struct point vector1, vector2, dest
// result goes into dest
#define CROSSPRODUCTp(vector1,vector2,dest)\
  CROSSPRODUCT( (vector1).x, (vector1).y, (vector1).z,\
                (vector2).x, (vector2).y, (vector2).z,\
                (dest).x, (dest).y, (dest).z )

// computes crossproduct of three vectors, which are specified by floating-point coordinates
// double x1,y1,z1,x2,y2,z2,x,y,z
// result goes into x,y,z
#define CROSSPRODUCT(x1,y1,z1,x2,y2,z2,x,y,z)\
\
  x = (y1) * (z2) - (y2) * (z1);\
  y = (x2) * (z1) - (x1) * (z2);\
  z = (x1) * (y2) - (x2) * (y1)

// normalizes vector dest
// struct point dest
// result returned in dest
// must declare a double variable called 'length' somewhere inside the scope of the NORMALIZE macrp
// macro will change that variable
#define pNORMALIZE(dest)\
\
  length = sqrt((dest).x * (dest).x + (dest).y * (dest).y + (dest).z * (dest).z);\
  (dest).x /= length;\
  (dest).y /= length;\
  (dest).z /= length;

// copies vector source to vector dest
// struct point source,dest
#define pCPY(source,dest)\
\
  (dest).x = (source).x;\
  (dest).y = (source).y;\
  (dest).z = (source).z;
  
// assigns values x,y,z to point vector dest
// struct point dest
// double x,y,z
#define pMAKE(x,y,z,dest)\
\
  (dest).(x) = (x);\
  (dest).(y) = (y);\
  (dest).(z) = (z);

// sums points src1 and src2 to dest
// struct point src1,src2,dest
#define pSUM(src1,src2,dest)\
\
  (dest).x = (src1).x + (src2).x;\
  (dest).y = (src1).y + (src2).y;\
  (dest).z = (src1).z + (src2).z;

// dest = src2 - src1
// struct point src1,src2,dest
#define pDIFFERENCE(src1,src2,dest)\
\
  (dest).x = (src1).x - (src2).x;\
  (dest).y = (src1).y - (src2).y;\
  (dest).z = (src1).z - (src2).z;

// mulitplies components of point src by scalar and returns the result in dest
// struct point src,dest
// double scalar
#define pMULTIPLY(src,scalar,dest)\
\
  (dest).x = (src).x * (scalar);\
  (dest).y = (src).y * (scalar);\
  (dest).z = (src).z * (scalar);

#endif


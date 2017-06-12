/*

  USC/Viterbi/Computer Science
  "cloth Cube" Assignment 1 starter code

*/

#include "cloth.h"
#include "physics.h"
#include <Eigen/Dense>

using namespace Eigen;

struct Box {
	point origin;
	double xLength;
	double yLength;
	double zLength;
};

struct Sphere {
	point origin;
	double radius;
};

void computeAcceleration(struct world *cloth, struct point a[10][10])
{
	// gamasutra/devil in blue dress

	//	cloth->
	// 1) stretch force
	// 2) shear and bend
	// 3) damping
	// 4) physics with other objects

	// NOTE: use IMPLICIT integration
	//	p(t) = p(t) + h * v(t + delta);
	// HUGE K, small h

	// k = stiffness constant
	// penalty force for collision from Assignment 1

	// energy function

	// gets the number of points for cloth
	int n = cloth->size * cloth->size;

	// rows and cols include (x, y, z) for n points and n constraints (n RIGID)
	// stretch: 4
	// shear: 4
	// bend: 4

	// iterate through all the triangles for stretch

	point xi, xj, xk;
	point diff;
	pDIFFERENCE(xj, xi, diff);
	pDIFFERENCE(xk, xi, diff);

	double wu; // unstretched is 1
	// wu = unstretched / actual distance

	//http://cg.alexandra.dk/?p=147

	// look at 4.1 for forces and derivative
	int size = (3 * n) + n + 1;

	double area = 0;
	double stretchU = 0;
	double stretchV = 0;

	// shear: a wu(x)T wv(x)

	//  H =	{mass,		dc/dq (transpose)	}
	//		{dc/dq,		0					}

	MatrixXd h(size, size);
	h.setZero();

	// set the mass matrix
	for (int i = 0; i < 3 * n; ++i)
	{
		h(i, i) = cloth->mass;
	}

	// set the n+1 constraints: dc/dq
	MatrixXd constraint(n + 1, 2 * n);
	constraint.setZero();

	for (int i = 0; i < n; ++i)
	{
		constraint(3 * n + i, 0) = 2 * 1;
	}

//	cloth->kElastic;
//	cloth->dElastic;

	// rigid constraint
	/*
	constraint(0, 0) = -2 * -cloth->p[1].y;
	constraint(0, 0 + 1) = -2 * -cloth->p[1].z;

	for (int i = 1; i < n; ++i)
	{
		int col = 2 * (i - 1);
		constraint(i, col) = 2 * (c->points[i].y - c->points[i + 1].y);
		constraint(i, col + 1) = 2 * (c->points[i].z - c->points[i + 1].z);
		constraint(i, col + 2) = -2 * (c->points[i].y - c->points[i + 1].y);
		constraint(i, col + 3) = -2 * (c->points[i].z - c->points[i + 1].z);
	}

	// assign constraints to h
	h.block(2 * n, 0, n + 1, 2 * n) = constraint;
	constraint.transposeInPlace();
	h.block(0, 2 * n, 2 * n, n + 1) = constraint;

	//	std::cout << h << "\nH\n\n";

	// creating matrix for RHS
	VectorXd rhs(size);
	rhs.setZero();

	// Apply Baumgarte stabilization
	// parameters for alpha and beta
	double alpha = c->alpha;
	double beta = c->beta;

	// put in the forces
	for (int i = 0; i < n; ++i)
	{
		// y force
		rhs(2 * i) = 0;

		// z force
		rhs(2 * i + 1) = -1 * c->mass; // gravitational force

		// damping force
		rhs(2 * i) -= alpha * c->v[i + 1].y;
		rhs(2 * i + 1) -= alpha * c->v[i + 1].z;
	}


	double base;
	double firstD;
	double secondD;

	// rigid constraint
	for (int i = 0; i < n; ++i)
	{
		base = pow(c->points[i].y - c->points[i + 1].y, 2) + pow(c->points[i].z - c->points[i + 1].z, 2) - pow(c->length, 2);

		// dc/dq * velocity
		firstD = 2 * (
			c->v[i].y * (c->points[i].y - c->points[i + 1].y) +
			c->v[i].z * (c->points[i].z - c->points[i + 1].z) -
			c->v[i + 1].y * (c->points[i].y - c->points[i + 1].y) -
			c->v[i + 1].z * (c->points[i].z - c->points[i + 1].z)
			);

		// dc'/dq * velocity
		secondD = 2 * (
			c->v[i].y * (c->v[i].y - c->v[i + 1].y) +
			c->v[i].z * (c->v[i].z - c->v[i + 1].z) -
			c->v[i + 1].y * (c->v[i].y - c->v[i + 1].y) -
			c->v[i + 1].z * (c->v[i].z - c->v[i + 1].z)
			);

		// multiplying velocity with dq/dt
		rhs((2 * n) + i) = -(secondD + (2 * beta * firstD) + (beta * beta * base));
	}
	//	std::cout << rhs << "\nRHS\n\n";

	// Finding the solution
*/

	// decompose h into u and v vectors
	JacobiSVD<MatrixXd> svd(h, ComputeThinU | ComputeThinV);

	// Truncates singular values into zero upon "solve()"
	svd.setThreshold(0.000001);

/*	VectorXd solution = svd.solve(rhs);

	// get the acceleration
	for (int i = 0; i < n; ++i)
	{
	c->a[i + 1].x = solution(3 * i);
	c->a[i + 1].y = solution(3 * i + 1);
		c->a[i + 1].z = solution(3 * i + 2);
	}*/
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

void Euler(struct world * cloth)
{
	// h = dt;

	int i, j, k;
	point a[10][10];
	for (i = 0; i <= 9; i++)
		for (j = 0; j <= 9; j++)
		{
			a[i][j].x = 0;
			a[i][j].y = 0;
			a[i][j].z = 0;
		}

	computeAcceleration(cloth, a);

	for (i = 0; i <= 9; i++)
		for (j = 0; j <= 9; j++)
		{
			
			cloth->p[i][j].x += cloth->dt * cloth->v[i][j].x;
			cloth->p[i][j].y += cloth->dt * cloth->v[i][j].y;
			cloth->p[i][j].z += cloth->dt * cloth->v[i][j].z;
			cloth->v[i][j].x += cloth->dt * a[i][j].x;
			cloth->v[i][j].y += cloth->dt * a[i][j].y;
			cloth->v[i][j].z += cloth->dt * a[i][j].z;
			
		}
}

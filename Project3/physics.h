/*

  USC/Viterbi/Computer Science

*/

#ifndef _PHYSICS_H_
#define _PHYSICS_H_

//void computeAcceleration(struct world * cloth, struct point a[8][8][8]);

// perform one step of Euler and Runge-Kutta-4th-order integrators
// updates the jello structure accordingly
void Euler(struct world * cloth);
//void RK4(struct world * cloth);

#endif


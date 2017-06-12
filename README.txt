CSCI 520, Cloth Simulation with Physics

Alex Wang

================

The project runs using OpenGL, specifically gl.h, glu.h, and glut.h

To run this project, you can either run file executable in the format under Debug folder:
Project3 [world filename]

To modify the parameters, you can create .w file, and modify it accordingly to the format provided in "world.w"


================

This project is implemented using Euler's method as a mode of integration.

To model the cloth itself, a mass-spring system is used to simulate physics and interactions with foreign objects.
The additional feature for this was to attach a spring that a point can reach within 2 edges.

The force simulated include spring (Hooke and damping force), gravitational force, and air resistance force.

Additionally, the cloth tracks the collision by calculating the future position in dt. The collision point is calculated and used to place a collision spring to prevent collision.
For collisions, the project support ground collision, sphere collisions, and cloth-to-cloth collision.
Each point is represented as a sphere that detects collisions with another, if this pair does NOT have a spring connected with each other.

Originally, the concept of the project started out with Baraff and Witkin paper as a basis for cloth simulation.
However, the approach of the project became based on collision detection and model described in https://graphics.stanford.edu/~mdfisher/cloth.html
to establish a simple implementation and support the features desired.
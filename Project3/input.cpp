/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "cloth.h"
#include "input.h"

/* Write a screenshot, in the PPM format, to the specified filename, in PPM format */
void saveScreenshot(int windowWidth, int windowHeight, char *filename)
{
  if (filename == NULL)
    return;

  // Allocate a picture buffer 
  Pic * in = pic_alloc(windowWidth, windowHeight, 3, NULL);

  printf("File to save to: %s\n", filename);

  for (int i=windowHeight-1; i>=0; i--) 
  {
    glReadPixels(0, windowHeight-i-1, windowWidth, 1, GL_RGB, GL_UNSIGNED_BYTE,
      &in->pix[i*in->nx*in->bpp]);
  }

  if (ppm_write(filename, in))
    printf("File saved Successfully\n");
  else
    printf("Error in Saving\n");

  pic_free(in);
}

/* converts mouse drags into information about rotation/translation/scaling */
void mouseMotionDrag(int x, int y)
{
  int vMouseDelta[2] = {x-g_vMousePos[0], y-g_vMousePos[1]};

  if (g_iRightMouseButton) // handle camera rotations
  {
    Phi += vMouseDelta[0] * 0.01;
    Theta += vMouseDelta[1] * 0.01;
    
    if (Phi>2*pi)
      Phi -= 2*pi;
    
    if (Phi<0)
      Phi += 2*pi;
    
    if (Theta>pi / 2 - 0.01) // dont let the point enter the north pole
      Theta = pi / 2 - 0.01;
    
    if (Theta<- pi / 2 + 0.01)
      Theta = -pi / 2 + 0.01;
    
    g_vMousePos[0] = x;
    g_vMousePos[1] = y;
  }

  if (g_iLeftMouseButton) // allows user to apply force with mouse direction
  {
  }

}

void mouseMotion (int x, int y)
{
  g_vMousePos[0] = x;
  g_vMousePos[1] = y;
}

void mouseButton(int button, int state, int x, int y)
{
  switch (button)
  {
    case GLUT_LEFT_BUTTON:
      g_iLeftMouseButton = (state==GLUT_DOWN);
      break;
    case GLUT_MIDDLE_BUTTON:
      g_iMiddleMouseButton = (state==GLUT_DOWN);
      break;
    case GLUT_RIGHT_BUTTON:
      g_iRightMouseButton = (state==GLUT_DOWN);
      break;
  }
 
  g_vMousePos[0] = x;
  g_vMousePos[1] = y;
}

// gets called whenever a key is pressed
void keyboardFunc (unsigned char key, int x, int y)
{
  switch (key)
  {
    case 27:
      exit(0);
      break;

    case 'e':
      Theta = pi / 6;
      Phi = pi / 6;
      viewingMode = 0;
      break;

    case 'v':
      viewingMode = 1 - viewingMode;
      break;

    case 'h':
      shear = 1 - shear;
      break;

    case 's':
      structural = 1 - structural;
      break;

    case 'b':
      bend = 1 - bend;
      break;

    case 'p':
      pause = 1 - pause;
      break;

    case 'z':
      R -= 0.2;
      if (R < 0.2)
        R = 0.2;
      break;

    case 'x':
      R += 0.2;
      break;

    case ' ':
		saveScreenToFile = 1 - saveScreenToFile;
      break;
  }
}
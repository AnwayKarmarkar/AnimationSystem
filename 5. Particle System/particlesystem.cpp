#include <GL/glut.h>
#include <stdio.h>
#include <math.h>
float texture[10];

//Total number of particles at one point on the screen
const int particleCount = 200;

typedef struct
{
	//Particle X Position
	double xPos;
	//Particle Y Position
	double yPos;
	//Particle Z Position
	double zPos;

	//Movement on the X axis
	double xVelocity;
	//Movement on the Z axis
	double zVelocity;

	//Red value for one particle
	float red;
	//Green value for one particle
	float green;

	//Variable that changes colour with time
	float colourDegree;

	//Angle of rotation
	double direction;
	//Acceleration Upwards
	double againstGravity;
	//Acceleration Downwards
	double withGravity;
	//How much we scale it
	double scaleZ;
}particleData;

//Array of particles
particleData Particle[particleCount];

//Initial values of the particles
void glCreateParticles ()
{
	int i;
	//For every single particle
	for (i = 1; i < particleCount; i++)
	{
		//Initial Particle Positions
		Particle[i].xPos = 0;
		Particle[i].yPos = -5;
		Particle[i].zPos = -5;

		//Randomize the X and Z movements
		Particle[i].xVelocity = (((((((2 - 1 + 1) * rand()%11) + 1) - 1 + 1) * rand()%11) + 1) * 0.005) - (((((((2 - 1 + 1) * rand()%11) + 1) - 1 + 1 ) * rand()%11) + 1) * 0.005);
		Particle[i].zVelocity = (((((((2 - 1 + 1) * rand()%11) + 1) - 1 + 1) * rand()%11) + 1) * 0.005) - (((((((2 - 1 + 1) * rand()%11) + 1) - 1 + 1 ) * rand()%11) + 1) * 0.005);

		//Colour
		Particle[i].red = 0.8;
		Particle[i].green = 0.5;
		Particle[i].colourDegree = 0.9;

		//Make every particle 1/4th the times its original size at the start
		Particle[i].scaleZ = 0.25;
		//Rotation angle = 0
		Particle[i].direction = 0;
		//Each acceleration is different so each particle rises to a different height
		Particle[i].againstGravity = ((((((8 - 5 + 2) * rand()%11) + 5 ) - 1 + 1) * rand()%11) + 1) * 0.02;
		//Particle has constant gravity acting on it
		Particle[i].withGravity = 0.0025;
	}
}


void UpdateParticles ()
{
	//For each frame
	int i;
	for (i = 1; i < particleCount; i++)
	{
		//Change Y position according to the gravitational acceleration
		Particle[i].yPos = Particle[i].yPos + Particle[i].againstGravity - Particle[i].withGravity;
		//Increase effect of gravity with each frame
		Particle[i].withGravity = Particle[i].withGravity +0.0025;

		//Update X and Z positions with the velocities
		Particle[i].xPos = Particle[i].xPos + Particle[i].xVelocity;
		Particle[i].zPos = Particle[i].zPos + Particle[i].zVelocity;

		//Make the rotation angle random to simulate random directions
		Particle[i].direction = Particle[i].direction + ((((((int)(0.5 - 0.1 + 0.1) * rand()%11) + 1) - 1 + 1) * rand()%11) + 1);
		//With each frame, the colour is changed
		Particle[i].colourDegree = Particle[i].colourDegree-0.008;

		//If the particle drops below a certain point,
		if (Particle[i].yPos < -5)
		{
			//Reset position to inital value
			Particle[i].xPos = 0;
			Particle[i].yPos = -5;
			Particle[i].zPos = -5;

			//Resent colour to initial value
			Particle[i].red = 0.8;
			Particle[i].green = 0.5;
			Particle[i].colourDegree = 0.9;

			//Resent direction and accelerations to inital values
			Particle[i].direction = 0;
			Particle[i].againstGravity = ((((((8 - 5 + 2) * rand()%11) + 5) - 1 + 1) * rand()%11) + 1) * 0.02;
			Particle[i].withGravity = 0.0025;
		}

		//Update colour
		Particle[i].red = 8 * (Particle[i].colourDegree*5);
		Particle[i].green = 5 * Particle[i].colourDegree/2;

		//Material values
		float kA[] = {Particle[i].red, Particle[i].green, 0.0, 1.0f};
		float kD[] = { Particle[i].red, Particle[i].green, 0.0, 1.0f };
		float kS[] = { 0.013f, 0.033f, 0.12f, 1.0f };
		float kE[] = { 0.1f, 0.1f, 0.1f, 1.0f };
		float material_Se = 1;

		glMaterialfv(GL_FRONT, GL_AMBIENT, kA);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, kD);
		glMaterialfv(GL_FRONT, GL_SPECULAR, kS);
		glMaterialfv(GL_FRONT, GL_EMISSION, kE);
		glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

		//Movement section
		glPushMatrix();
		glTranslatef (Particle[i].xPos, Particle[i].yPos, Particle[i].zPos);
		glRotatef (Particle[i].direction - 90, 0, 0, 1);

		//Make particle bigger with each system
		glScalef (Particle[i].scaleZ, Particle[i].scaleZ, Particle[i].scaleZ);

		//Render sphere
		glutSolidSphere(0.7,10,10);

		glPopMatrix();

	}
}

//Render Function
void display (void)
{
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnable(GL_DEPTH_TEST);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	float AmbientLight[] = { 0.04, 0.04f, 0.04f, 0.0f };
	float DiffuseLight[] = { 0.07f, 0.07f, 0.07f, 0.0f };
	float SpecularLight[] = { 0.01f, 0.01f, 0.01f, 0.0f };
	float LightPosition[] = { 5.0f, 5.0f, 5.0f, 0.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT, AmbientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, DiffuseLight);
	glLightfv(GL_LIGHT0, GL_SPECULAR, SpecularLight);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);


	// modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0, 15.0, 15.0 , 0.0, 3.0, 0.0, 	0.0, 1.0, 0.0);

    UpdateParticles();
	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	// swap back and front buffers
	glutSwapBuffers();
}

// timer
void timer(int value)
{
	// re - render scene
	glutPostRedisplay();


	// reset timer
	glutTimerFunc(16, timer, 0);
}


void reshape(int w, int h)
{
	// viewport
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	// projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(40.0, (float)w / (float)h, 1.0, 50.0);
}

int main (int argc, char **argv)
{
    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize (800, 600);
    glutInitWindowPosition (100, 100);
    glutCreateWindow("Lab5");
    glCreateParticles();
    glutDisplayFunc (display);
    glutReshapeFunc (reshape);
    glutTimerFunc(16, timer, 0);

    glutMainLoop ();
    return 0;
}


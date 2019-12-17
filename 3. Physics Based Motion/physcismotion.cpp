/*
 * lab3.cpp
 *
 *  Created on: Nov 11, 2019
 *      Author: Anway
 */
#include <math.h>
#include<stdio.h>
#include<iostream>
#include <GL/glut.h>

using namespace std;

// Global Variable Declaration

// Total number of balls
int number = 8;

//Ball Starting Locations
float ballStartLoc[8][3] = { { -4.0f, 0.0, 0.0f },{ -5.0f, 0.0, 0.6f },{ -5.0f, 0.0, -0.6f },{ -6.1f, 0.0, 1.0f },
						{ -6.0f, 0.0, 0.0f },{ -6.0f, 0.0, -1.1f },{ -6.0f, 10.0f, 0.6f },{ 9.0f, 0.0f, 0.0f }};

//Ball Starting Velocities
float ballVelo[8][3] = { { 0.0, 0, 0 },{ 0, 0, 0 },{ 0, 0, 0 },{ 0, 0, 0 },
						{ 0, 0, 0 },{ 0, 0, 0 },{ 0, 5, 0 },{ -5, 0, 0 }};

//Ball Positions
float ballPosn[8][3] = { 0 };

//Timer counter
float t;

//The Matrix for each ball. 8 balls have a 4 * 4 matrix each
float ballMat[8][4][4];

// Normalize a vector for getting a value between 0 and 1
void Normalize(float vec[3])
{
	float sqSum = vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
	if (sqSum != 0) // avoid being divided by 0
	{
		float temp = sqrt(sqSum);
		vec[0] = vec[0] / temp;
		vec[1] = vec[1] / temp;
		vec[2] = vec[2] / temp;
	}
}

//Vector Dot Product
float DotProduct(float v1[3], float v2[3])
{
	float a,b,c,result;
	a = v1[0] * v2[0];
	b = v1[1] * v2[1];
	c = v1[2] * v2[2];
	result = a + b + c;

	return result;
}

//Distance Function
float Distance(float ball1[3], float ball2[3])
{

	float sq1 = (ball1[0] - ball2[0]) * (ball1[0] - ball2[0]);
	float sq2 = (ball1[1] - ball2[1]) * (ball1[1] - ball2[1]);
	float sq3 = (ball1[2] - ball2[2]) * (ball1[2] - ball2[2]);
	float sum = sq1 + sq2 + sq3;
	float distance = sqrt(sum);

	return distance;
}

/* This function provides the Collision velocity components
 * When a collision occurs, there are two components to the velocity of the collision
 * One component is along the vector that connects the centres of the two balls
 * Other component is perpendicular to the first component
 * The second ball will also have similar components
 */
void GetCollisionComponents(int ball1, int ball2, float a[3], float b[3], float CollisionVector[3])
{
	// Calculate the vector between to centres of the ball
	CollisionVector[0] = ballPosn[ball2][0] - ballPosn[ball1][0];
	CollisionVector[1] = ballPosn[ball2][1] - ballPosn[ball1][1];
	CollisionVector[2] = ballPosn[ball2][2] - ballPosn[ball1][2];

	//Normalize the vector
	Normalize(CollisionVector);

	//Angle between the collision vector and the velocity direction of the ball
	float temp = DotProduct(CollisionVector, ballVelo[ball1]);

	//a is the component of the velocity along the collision vector.
	a[0] = temp*CollisionVector[0];
	a[1] = temp*CollisionVector[1];
	a[2] = temp*CollisionVector[2];

	//b is the component of the velocity perpendicular to the collision vector.
	b[0] = ballVelo[ball1][0] - a[0];
	b[1] = ballVelo[ball1][1] - a[1];
	b[2] = ballVelo[ball1][2] - a[2];
}

/*
 * This function detects collision between two balls
 * If the distance between two balls is less than the sum of the two radii, collision has occured
 * The radius of the balls is 0.5
 *
 * After collision, the resulting velocity change is calculated for both balls
 */
void BallCollision(int ball)//ball is the current ball that is being checked
{
	//Detect collision between current ball and every other ball
	for (int i = ball + 1; i < number; i++)
	{
		//collision detection condition
		if (Distance(ballPosn[ball], ballPosn[i])<=1.0)//i is the ball that the current ball has collided with
		{
			float ballAxis[3], a1[3], b1[3], a2[3], b2[3], a1Final[3], a2Final[3];;

			//Get the collision vector data for the current ball
			GetCollisionComponents(ball,i,a1,b1,ballAxis);

			//Get the collision vector data for the ball that the current ball collided with
			GetCollisionComponents(i,ball,a2,b2,ballAxis);

			for (int j = 0; j < 3; j++)
			{
				//Final velocity for the current ball along the collision vector
				a1Final[j] = (a1[j] - (a1[j] - a2[j]));

				//Final velocity for the collided ball along the collision vector
				a2Final[j] = (a2[j] - (a2[j] - a1[j]));

				//Final Velocity for ball 1
				ballVelo[ball][j] = (a1Final[j] + b1[j]);
				//Final Velocity for ball 2
				ballVelo[i][j] = (a2Final[j] + b2[j]);
			}
		}
	}
}

/*
 * This function detects and calculates the balls' collision with the floor and walls of the room
 * The X, Y or Z velocities are mirrored according to the element that the balls collide with
 * The resultant velocity is reduced by the coefficient of restitution every time it collides with a wall
 */
void RoomCollision(int ball)//ball is the ball that has collided with the room
{
	//Coefficient of Restitution
	float  coeffOfRest = 0.89;

	//The ball's Y coordinate drops below 0, This means that it has collided with the floor
	if (ballPosn[ball][1]<0)
	{
		//To prevent the ball from dropping below the floor, the position is moved to 0
		ballPosn[ball][1]=0;
		//The Y component of the velocity is mirrored
		ballVelo[ball][1] = -coeffOfRest*ballVelo[ball][1];
	}

	//Collision with the right or left walls
	//If the X coordinate exceeds  9.5 or drops below -9.5 then the ball has collided
	if (ballPosn[ball][0]>9.5 || ballPosn[ball][0]<-9.5)
	{
		//To prevent the ball from moving beyond the wall confines because of collision occurring between frames
		if(ballPosn[ball][0] < -5)
		{
			ballPosn[ball][0] = -9.5;
		}
		if(ballPosn[ball][0] > 5)
		{
			ballPosn[ball][0] = 9.5;
		}

		//The X component of the velocity is mirrored and reduced by a factor of the coefficient of restitution
		ballVelo[ball][0] = -coeffOfRest*ballVelo[ball][0];
	}

	//Collision with the right or left walls
	//If the Z coordinate exceeds  9.5 or drops below -9.5 then the ball has collided
	if (ballPosn[ball][2]>9 || ballPosn[ball][2]<-9)
	{
		//To prevent the ball from moving beyond the wall confines because of collision occurring between frames
		if(ballPosn[ball][2] < -5)
		{
			ballPosn[ball][2] = -9;
		}
		if(ballPosn[ball][2] > 5)
		{
			ballPosn[ball][2] = 9;
		}
		//The Z component of the velocity is mirrored and reduced by a factor of the coefficient of restitution
		ballVelo[ball][2] = -coeffOfRest*ballVelo[ball][2];
	}
}

/*
 * This function calculates the velocity of the ball in every frame, at every time increment
 * The collisions are also resolved here
 * The velocity is used to calculate the position of the ball in the next frame
 * This position is put into the model matrix for the bal
 */
void BallMove(int ball){

	//Collision portion
	RoomCollision(ball);
	BallCollision(ball);

	float nextPos[8][3];
	float nextVelo[8][3];

	//Time is incremented by this value every frame
	float timeIncrement = 0.03f;

	//Gravity
	float gravityVector[3] = { 0, -2.0, 0 };

	for (int i = 0; i<3; i++)
	{
		//Velocity for the next frame
		//This is calculated using the current velocity and the acceleration for the increment in time
		//so acceleration is the change in velocity for every time increment
		nextVelo[ball][i] = ballVelo[ball][i] + gravityVector[i] * timeIncrement;
		ballVelo[ball][i] = nextVelo[ball][i];

		//Similar to how the velocity is calculated using acceleration, The position is calculated using velocity
		//The previous position is changed by the velocity for the time increment
		//Velocity is the change in position
		nextPos[ball][i] = ballPosn[ball][i] + ballVelo[ball][i] * timeIncrement;
		ballPosn[ball][i] = nextPos[ball][i];

		//The position is loaded into the ball model matrix
		ballMat[ball][3][i] = ballPosn[ball][i];
	}
}

/*
 * Render the balls
 * Each matrix is pushed, loaded and popped from the stack, and them the matrix is rendered
 */
void BallAnimation()
{
	//Matrix to be loaded for rendering
	float M[16] = { 0 };
	for (int k = 0; k < number; k++)
	{
		glPushMatrix();

		//Calculate the ball position
		BallMove(k);
		for(int i=0;i<4;i++)
		{
			for(int j=0;j<4;j++)
			{
				//Load the ball matrix
				M[(4*i) + j] = ballMat[k][i][j];
			}
		}
		//Multiply the matrix
		glMultMatrixf(M);

		//Render the sphere
		glutSolidSphere(0.5, 20, 20);
		glPopMatrix();
	}
}

/*
 * Render the room
 * Each wall is rendered separately
 */
void RoomRender()
{
	//Floor
	glColor3f(0.3, 0.15, 0.35);
	glBegin(GL_QUADS);
		glVertex3f(-10,-1,-10);
		glVertex3f(10,-1,-10);
		glVertex3f(10,-1,10);
		glVertex3f(-10,-1,10);
	glEnd();

	//Left Wall
	glColor3f(1.0, 0.4, 0.5);
	glBegin(GL_QUADS);
		glVertex3f(-10,-1,10);
		glVertex3f(-10,5,10);
		glVertex3f(-10,5,-10);
		glVertex3f(-10,-1,-10);
	glEnd();

	//Right Wall
	glColor3f(1.0, 0.4, 0.5);
	glBegin(GL_QUADS);
		glVertex3f(10,-1,10);
		glVertex3f(10,5,10);
		glVertex3f(10,5,-10);
		glVertex3f(10,-1,-10);
	glEnd();

	//Back Wall
	glColor3f(0.0, 0.9, 0.5);
	glBegin(GL_QUADS);
		glVertex3f(-10,-1,-10);
		glVertex3f(-10,5,-10);
		glVertex3f(10,5,-10);
		glVertex3f(10,-1,-10);
	glEnd();

	//Front Wall
	glColor3f(0.0, 0.9, 0.5);
	glBegin(GL_QUADS);
		glVertex3f(-10,-1,10);
		glVertex3f(-10,5,10);
		glVertex3f(10,5,10);
		glVertex3f(10,-1,10);
	glEnd();
}

/*
 * Timer function
 * Refreshes the scene at 60 frames per second
 * the timer value is increased as well
 */
void timer(int value)
{
	glutPostRedisplay();
	t = t + 0.005;
	glutTimerFunc(16, timer, 0);
}

//Contains the material and lighting data for the balls
void BallColour()
{
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	float AmbientLight[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	float DiffuseLight[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	float SpecularLight[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	float LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT, AmbientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, DiffuseLight);
	glLightfv(GL_LIGHT0, GL_SPECULAR, SpecularLight);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	float kA[] = {0.3f, 1.0f, 1.0f, 1.0f };
	float kD[] = { 0.23f, 0.67f, 0.54f, 1.0f };
	float kS[] = { 0.13f, 0.33f, 0.52f, 1.0f };
	float kE[] = { 0.1f, 0.0f, 0.1f, 1.0f };
	float material_Se = 15;

	glMaterialfv(GL_FRONT, GL_AMBIENT, kA);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, kD);
	glMaterialfv(GL_FRONT, GL_SPECULAR, kS);
	glMaterialfv(GL_FRONT, GL_EMISSION, kE);
	glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

	//Render the ball
	BallAnimation();

	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
}

// Render Function
void render(void)
{

	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(15 * sin(t), 20.0, 15.0 * sin(1-t),   0.0, 0.0, 0.0,      0.0, 1.0, 0.0);

	BallColour();
	RoomRender();

	glFlush();
	glutSwapBuffers();
}

//Set up the viewport
void reshape(int w, int h)
{
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(70.0, (float)w / (float)h, 1.0, 50.0);
}


//Main Function
int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Lab 3");

	//Initialise the Ball matrices
	for (int i = 0; i<number; i++)
	{
		ballMat[i][0][0] = 1.0f;
		ballMat[i][1][1] = 1.0f;
		ballMat[i][2][2] = 1.0f;
		ballMat[i][3][3] = 1.0f;
		for (int j = 0; j<3; j++)
		{
			ballMat[i][3][j] = ballStartLoc[i][j];
			ballPosn[i][j] = ballStartLoc[i][j];
		}
	}

	glutDisplayFunc(render);
	glutReshapeFunc(reshape);
	glutTimerFunc(16, timer, 0);
	glutMainLoop();

	return 0;
}


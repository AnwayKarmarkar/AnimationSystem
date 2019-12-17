/*
 * lab2.cpp
 *
 *  Created on: Oct 21, 2019
 *      Author: Anway
 */


#include <math.h>
#include<stdio.h>
#include<iostream>
#include <GL/glut.h>

using namespace std;


// Global Variable Declaration

// Key Frames
int currentKeyFrame = 0; //Current Key Frame Index for the Splines
int totalFrames = 7; //Total number of spline key frames


//Time Variables
float t = 0;
int loopIndex = 0;


//Vectors for direction of movement
//This is to calculate the direction in which the body will face
float tangentVec[3] = { 0 };
//This always points up as body will walk on a horizontal plane
float normalVec[3] = { 0 };
//This is the perpendicular vector to the body direction and the vertical vector
float binormalVec[3] = { 0 };


//The final M matrix for torso, Left Leg and Right Leg
float BodyMovementMat[4][4] = { 0 }; //torso

//Temporary Matrix
float tempM[3] = {0};

//The Model Matrices for the various body parts
float leftLegMat[4][4] = { 0 }; //Left Leg
float rightLegMat[4][4] = { 0 }; //Right Leg
float leftShinMat[4][4] = { 0 }; //Left Lower Leg
float rightShinMat[4][4] = { 0 }; //Right Lower Leg
float spineMat[4][4] = { 0 }; //Spine
float shoulderMat[4][4] = { 0 }; //Shoulder
float leftArmMat[4][4] = { 0 }; //Left Arm
float rightArmMat[4][4] = { 0 }; //Right Arm
float headMat[4][4] = { 0 }; //Head

// Basis Matrix for the Spline
float splineBasisMat[4][4] ={ { -1.0f / 6.0f, 3.0f / 6.0f, -3.0f / 6.0f, 1.0f / 6.0f},
		{3.0f / 6.0f, -6.0f / 6.0f, 0.0f / 6.0f, 4.0f / 6.0f},
		{-3.0f / 6.0f, 3.0f / 6.0f, 3.0f / 6.0f, 1.0f / 6.0f},
		{1.0f / 6.0f, 0.0f / 6.0f, 0.0f / 6.0f, 0.0f / 6.0f }};

// Key Frames for the Spline Points
float keyFramePoints[7][3] = { { -8, 0, -20 },{ 3, 0, -20 },{ -5, 0, -10 },{ 5, 0, -10 },
								{ 3, 0, -5 },{-3,0,-5},{ 1, 0, -3 } };

//Blending Function to calculate interpolated values for X Y Z positions and Rotations for the body
float Blend(float T[4], float MS[4][4], float G[4])
{
	float temp[4];
	for(int i=0;i<4;i++)
	{
		float temp1 = 0;
		for(int j=0;j<4;j++)
		{
			temp1+=T[j]*MS[i][j];
		}
		temp[i]=temp1;
	}

	float returnVal=0;
	for(int i=0;i<4;i++)
	{
		returnVal+= temp[i] * G[i];
	}
	return returnVal;
}




// Matrix Multiply : Multiply two 4*4 matrices
void MatMul(float A[4][4], float B[4][4], float MResult[4][4])
{
    int i, j, k;
    for(i=0;i<4;i++)
    {
		for(j=0;j<4;j++)
		{
			float temp=0;
			for(k=0;k<4;k++)
			{
				temp+=A[k][j]*B[i][k];
			}
			MResult[i][j] = temp;
		}
    }
}




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




//Cross Product of two vectors
void CrossProduct(float v1[3], float v2[3], float result[3])
{
	result[0] = v1[1] * v2[2] - v1[2] * v2[1];
	result[1] = v1[2] * v2[0] - v1[0] * v2[2];
	result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}




//This function calculates the position and orientation of the whole torso.
//This generates the matrix that is used to move all the individual body parts as a whole unit.
void BodyPosCalc(float keyFrames[7][3], float sMat[4][4])
{
	//Time Matrix. This is used to calculate the position of the body
	float tMat[4] = { t*t*t, t*t, t, 1 };

	//Time Matrix. This is the first differential of the Time Matrix defined above.
	//This changes the direction in which the body points according to the position of the key frame points.
	float tMatTangent[4] = { 3*t*t, 2*t, 1, 0 };

	//Generate position where the body will be in the next time frame
	//Also generate the direction in which the body points in the next time frame
	for (int i = 0; i < 3; i++)
	{
		//4 reference points for the geometry matrix
		float gMat[4] = { keyFrames[currentKeyFrame][i],
				//The points are wrapped around so that loop can continue on without interruption
				keyFrames[(currentKeyFrame + 1) % totalFrames][i],
				keyFrames[(currentKeyFrame + 2) % totalFrames][i],
				keyFrames[(currentKeyFrame + 3) % totalFrames][i] };

		tempM[i] = Blend(tMat, sMat, gMat);
		tangentVec[i] = Blend(tMatTangent, sMat, gMat);
	}

	// Normalize the tangent vector
	Normalize(tangentVec);

	// Loop starts from the beginning
	if (currentKeyFrame == 0)
	{
		//This vector will be on the same plane as the tangent vector
		float TempVector[3] = { 1, 0, 0 };
		Normalize(TempVector);

		/*Cross product between tangent vector and tempVector
		Because they are on the same plane, the cross product will generate a vector that points vertically
		In the positive Y direction*/
		CrossProduct(tangentVec, TempVector, normalVec);
		Normalize(normalVec);

		/*The cross product between the tangent and normal vector will generate a vector that points
		perpendicularly sideways to the tangent vector*/
		CrossProduct(normalVec, tangentVec, binormalVec);
		Normalize(binormalVec);

		/*
		 * These three vectors, tangent, normal and binormal are used to change the direction in which the body
		 * will point. This will help to turn the character realistically
		 */
	}
	else // loop does not start from the beginning
	{
		CrossProduct(tangentVec, binormalVec, normalVec);
		Normalize(normalVec);
		CrossProduct(normalVec, tangentVec, binormalVec);
		Normalize(binormalVec);
	}

	// Interpolation Matrix
	BodyMovementMat[0][0] = tangentVec[0]; 	BodyMovementMat[0][1] = normalVec[0]; BodyMovementMat[0][2] = binormalVec[0]; BodyMovementMat[0][3] = 0;
	BodyMovementMat[1][0] = tangentVec[1]; BodyMovementMat[1][1] = normalVec[1]; BodyMovementMat[1][2] = binormalVec[1];	BodyMovementMat[1][3] = 0;
	BodyMovementMat[2][0] = tangentVec[2]; BodyMovementMat[2][1] = normalVec[2]; BodyMovementMat[2][2] = binormalVec[2]; BodyMovementMat[2][3] = 0;
	BodyMovementMat[3][0] = tempM[0]; BodyMovementMat[3][1] = tempM[1]; BodyMovementMat[3][2] = tempM[2]; BodyMovementMat[3][3] = 1;

}




/* Pelvis Animation :
 * This is the anchor of the body
 * The entire skeletal structure us anchored to the pelvis
 * The Pelvis itself rotates around the Y axis mimicking the motion of a real pelvis
 *
 */
void PelvisAnimation()
{
	BodyPosCalc(keyFramePoints, splineBasisMat);

	float tRotMat[4][4] = { 0 };

	//Amount of rotation we want for the pelvis about the Y axis
	float pelvisAngle = -(sin(4 * 3.14*t - 3.14 / 2)*3.14) / 4;

	//The rotation matrix about the Y axis
	float pelvisRotationMat[4][4] = {{ cos(pelvisAngle), 0, sin(pelvisAngle), 0},
						{0, 1, 0, 0},
						{-sin(pelvisAngle), 0, cos(pelvisAngle), 0},
						{0, 0, 0, 1} };

	//Combining the body position and orientation matrix and Pelvis rotation matrix into the tRotMat
	MatMul(BodyMovementMat, pelvisRotationMat, tRotMat);

	//Convert 4*4 into straight 16 array
	float tM[16] = { 0 };
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			tM[(4*i) + j] = tRotMat[i][j];
		}
	}

	//Load the model matrix and render.
	glLoadMatrixf(tM);
	glScalef(0.3f, 0.3f, 1.0f);
	glutSolidCube(1.0);
}

/* Left Leg animation :
 * This is the left leg animation.
 * This has two parts : The thigh and shin
 * The shin rotates about the thigh and the thigh about the pelvis.
 * The shin uses the matrix generated for the thigh as a base point and rotates from there
 * This is used to generate the motion of a knee joint
 * The thigh also moves front and back along with the rotation of the pelvis.
 * This makes the motion more realistic
 */
void LLegAnimation()
{
	//This translation matrix is used to move the leg vertically downward from the origin point
	float vertThighMat[4][4] = { {1, 0, 0, 0},
						{0, 1, 0, 0},
						{0, 0, 1, 0},
						{0, -0.6, 0, 1 }};

	//Angle that the leg rotates by
	float legAngle = (sin(4 * 3.14*t - 3.14 / 2)*3.14) / 4;

	//Rotation Matrix about the Z axis
	float rotThighMat[4][4] = {{ cos(legAngle), sin(legAngle), 0, 0},
						{-sin(legAngle), cos(legAngle), 0, 0},
						{0, 0, 1, 0},
						{legAngle/2, 0, 0, 1} }; // The Leg is moved along the X axis to match rotation of the pelvis

	// This translation matrix is used to move the leg to an horizontal offset to mirror the right leg
	float horizThighMat[4][4] = {{ 1, 0, 0, 0},
						{0, 1, 0, 0},
						{0, 0, 1, 0},
						{0, 0, 0.4, 1 }};

	//These three matrix multiplications are used to combine the previous transformations into the body movement matrix
	MatMul(BodyMovementMat, rotThighMat, leftLegMat);
	MatMul(leftLegMat, vertThighMat, leftLegMat);
	MatMul(leftLegMat, horizThighMat, leftLegMat);

	//4*4 converted to straight 16
	float loadMat1[16];
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			loadMat1[(4*i) + j] = leftLegMat[i][j];
		}
	}

	//Load and render
	glLoadMatrixf(loadMat1);
	glScalef(0.3f, 1.5f, 0.3f);
	glutSolidCube(1.0);








	//This following section will be for calculating the shin rotation

	//Move the shin below the thigh
	float vertShinMat[4][4] = {{ 1, 0, 0, 0},
						{0, 1, 0, 0},
						{0, 0, 1, 0},
						{0, -0.5, 0, 1} };

	//Shin rotation angle
	float shinAngle = 0;

	//These conditions make sure the knee joint is not doubly rotating
	if(legAngle <= 0)
	{
		shinAngle = legAngle*1.5; //This is when the leg is moving backward, so the knee can bend
	}
	else
	{
		shinAngle = 0; //This is when the leg moves forward, so the knee straightens
	}

	//Rotation about the knee (Z axis)
	float rotShinMat[4][4] = { {cos(shinAngle), sin(shinAngle), 0, 0},
						{-sin(shinAngle), cos(shinAngle), 0, 0},
						{0, 0, 1, 0},
						{0, -0.7, 0, 1 }};


	//Matrix multiplications to combine the translation and rotation
	MatMul(leftLegMat, rotShinMat, leftShinMat);
	MatMul(leftShinMat, vertShinMat, leftShinMat);

	//4*4 converted to straight 16
	float loadMat2[16];
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			loadMat2[(4*i) + j] = leftShinMat[i][j];
		}
	}

	//Load and render
	glLoadMatrixf(loadMat2);
	glScalef(0.3f, 1.0f, 0.3f);
	glutSolidCube(1.0);

}





/* Right Leg animation :
 * This is the left leg animation.
 * This has two parts : The thigh and shin
 * The shin rotates about the thigh and the thigh about the pelvis.
 * The shin uses the matrix generated for the thigh as a base point and rotates from there
 * This is used to generate the motion of a knee joint
 * The thigh also moves front and back along with the rotation of the pelvis.
 * This makes the motion more realistic
 */
void RLegAnimation()
{
	//This translation matrix is used to move the leg vertically downward from the origin point
	float vertThighMat[4][4] = {{ 1, 0, 0, 0},
						{0, 1, 0, 0},
						{0, 0, 1, 0},
						{0, -0.6, 0, 1 }};

	//Angle that the leg rotates by
	float legAngle = (sin(4*3.14*t-3.14/2)*3.14)/4;

	//Rotation Matrix about the Z axis
	float rotThighMat[4][4] = { {cos(-legAngle), sin(-legAngle), 0, 0},
						{-sin(-legAngle), cos(-legAngle), 0, 0},
						{0, 0, 1, 0},
						{-legAngle/2, 0, 0, 1} };

	// This translation matrix is used to move the leg to an horizontal offset to mirror the left leg
	float horizThighMat[4][4] = { {1, 0, 0, 0},
						{0, 1, 0, 0},
						{0, 0, 1, 0},
						{0, 0, -0.4, 1} };


	//These three matrix multiplications are used to combine the previous transformations into the body movement matrix
	MatMul(BodyMovementMat, rotThighMat, rightLegMat);
	MatMul(rightLegMat, vertThighMat, rightLegMat);
	MatMul(rightLegMat, horizThighMat, rightLegMat);

	//4*4 converted to straight 16
	float loadMat1[16];
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			loadMat1[(4*i) + j] = rightLegMat[i][j];
		}
	}

	//Load and render
	glLoadMatrixf(loadMat1);
	glScalef(0.3f, 1.5f, 0.3f);
	glutSolidCube(1.0);





	//This following section will be for calculating the shin rotation

	//Move the shin below the thigh
	float vertShinMat[4][4] = {{ 1, 0, 0, 0},
						{0, 1, 0, 0},
						{0, 0, 1, 0},
						{0, -0.5, 0, 1} };

	//Shin rotation angle
	float shinAngle = 0;

	//These conditions make sure the knee joint is not doubly rotating
	if(legAngle <= 0)
	{
		shinAngle = 0; //This is when the leg moves forward, so the knee straightens
	}
	else
	{
		shinAngle = -legAngle*1.5;  //This is when the leg is moving backward, so the knee can bend
	}

	//Rotation about the knee (Z axis)
	float rotShinMat[4][4] = {{ cos(shinAngle), sin(shinAngle), 0, 0},
						{-sin(shinAngle), cos(shinAngle), 0, 0},
						{0, 0, 1, 0},
						{0, -0.7, 0, 1 }};

	//Matrix multiplications to combine the translation and rotation
	MatMul(rightLegMat, rotShinMat, rightShinMat);
	MatMul(rightShinMat, vertShinMat, rightShinMat);

	//4*4 converted to straight 16
	float loadMat2[16];
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			loadMat2[(4*i) + j] = rightShinMat[i][j];
		}
	}

	//Load and render
	glLoadMatrixf(loadMat2);
	glScalef(0.3f, 1.0f, 0.3f);
	glutSolidCube(1.0);
}

/*Spine Animation :
 * The spine follows the pelvis in motion
 *
 * Shoulder Animation :
 * The shoulder moves out of sync with the pelvis
 * The rotation is mirrored but is about the same axis
 *
 *Head Animation :
 * The head is simply moved to the correct position and moved along with the body
 */
void SpineShoulderHeadAnimation()
{
	// Spine is moved up from the pelvis
	float vertSpineMat[4][4] = {{ 1, 0, 0, 0},
						{0, 1, 0, 0},
						{0, 0, 1, 0},
						{0, 1.0, 0, 1 }};

	//Combined body matrix and spine
	MatMul(BodyMovementMat, vertSpineMat, spineMat);

	//4*4 converted to straight 16
	float loadMat1[16];
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			loadMat1[(4*i) + j] = spineMat[i][j];
		}
	}

	//Load and render
	glLoadMatrixf(loadMat1);
	glScalef(0.3f, 2.0f, 0.3f);
	glutSolidCube(1.0);







	//Following code is for the shoulder animation

	//Angle by which the shoulder rotates around the Y axis
	float shoulderAngle = (sin(4 * 3.14*t - 3.14 / 2)*3.14) / 16;

	//Rotation about the Y axis
	float rotShoulderMat[4][4] = {{ cos(shoulderAngle), 0, sin(shoulderAngle), 0},
						{0, 1, 0, 0},
						{-sin(shoulderAngle), 0, cos(shoulderAngle), 0},
						{0, 0, 0, 1} };

	//Move the shoulder to the correct position
	float vertShoulderMat[4][4] = {{ 1, 0, 0, 0},
						{0, 1, 0, 0},
						{0, 0, 1, 0},
						{0, 2.0, 0, 1 }};

	//combine the translation and rotation matrices into the body movement
	MatMul(BodyMovementMat, vertShoulderMat, shoulderMat);
	MatMul(shoulderMat, rotShoulderMat, shoulderMat);

	//4*4 converted to straight 16
	float loadMat2[16];
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			loadMat2[(4*i) + j] = shoulderMat[i][j];
		}
	}

	//Load and render
	glLoadMatrixf(loadMat2);
	glScalef(0.3f, 0.3f, 1.5f);
	glutSolidCube(1.0);







	//Following code is for the shoulder animation
	//Head moved vertically upward above the spine
	float HeadT[4][4] = {{ 1, 0, 0, 0},
						{0, 1, 0, 0},
						{0, 0, 1, 0},
						{0, 2.9, 0, 1 }};

	//Combine the body movement and head
	MatMul(BodyMovementMat, HeadT, headMat);

	//4*4 converted to straight 16
	float loadMat3[16];
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			loadMat3[(4*i) + j] = headMat[i][j];
		}
	}

	//Load and render
	glLoadMatrixf(loadMat3);
	glScalef(0.5f, 0.5f, 0.5f);
	glutSolidSphere(1.0,100,100);
}

/*
 * Left arm animation :
 * This is the same as that of right leg
 * Except it is moved vertically and horizontally to the correct position on the shoulder
 */
void LArmAnimation()
{
	float vertArmMat[4][4] = { {1, 0, 0, 0},
						{0, 1, 0, 0},
						{0, 0, 1, 0},
						{0, -0.7 , 0, 1 }};

	float armAngle = -(sin(4 * 3.14*t - 3.14 / 2)*3.14) / 4;

	float rotArmMat[4][4] = {{ cos(armAngle), sin(armAngle), 0, 0},
						{-sin(armAngle), cos(armAngle), 0, 0},
						{0, 0, 1, 0},
						{armAngle/16, 0, 0, 1} };

	float horizArmMAt[4][4] = {{ 1, 0, 0, 0},
						{0, 1, 0, 0},
						{0, 0, 1, 0},
						{0, 0, 0.6, 1 }};

	MatMul(shoulderMat, rotArmMat, leftArmMat);
	MatMul(leftArmMat, vertArmMat, leftArmMat);
	MatMul(leftArmMat, horizArmMAt, leftArmMat);

	float loadMat1[16];
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			loadMat1[(4*i) + j] = leftArmMat[i][j];
		}
	}

	glLoadMatrixf(loadMat1);
	glScalef(0.3f, 1.5f, 0.3f);
	glutSolidCube(1.0);
}




/*
 * Right arm animation :
 * This is the same as that of left leg
 * Except it is moved vertically and horizontally to the correct position on the shoulder
 */
void RArmAnimation()
{
	float vertArmMat[4][4] = { {1, 0, 0, 0},
						{0, 1, 0, 0},
						{0, 0, 1, 0},
						{0, -0.7 , 0, 1 }};

	float armAngle = (sin(4 * 3.14*t - 3.14 / 2)*3.14) / 4;

	float rotArmMAt[4][4] = {{ cos(armAngle), sin(armAngle), 0, 0},
						{-sin(armAngle), cos(armAngle), 0, 0},
						{0, 0, 1, 0},
						{armAngle/16, 0, 0, 1} };

	float horizArmMat[4][4] = {{ 1, 0, 0, 0},
						{0, 1, 0, 0},
						{0, 0, 1, 0},
						{0, 0, -0.6, 1 }};

	MatMul(shoulderMat, rotArmMAt, rightArmMat);
	MatMul(rightArmMat, vertArmMat, rightArmMat);
	MatMul(rightArmMat, horizArmMat, rightArmMat);

	float loadMat1[16];
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			loadMat1[(4*i) + j] = rightArmMat[i][j];
		}
	}

	glLoadMatrixf(loadMat1);
	glScalef(0.3f, 1.5f, 0.3f);
	glutSolidCube(1.0);
}

// timer : This function provides values used to interpolate and calculate the body position and orientation at each frame
void timer(int value)
{
	// re - render scene
	glutPostRedisplay();

	// Set time increase by 0.01, changing the value of points from 0 to 2
	t = t + 0.01;
	if (t >= 1)
	{
		t = 0;

		//Check if current key frame is still within the limit of the total number of key frames
		if (currentKeyFrame < totalFrames - 1)
		{
			currentKeyFrame++;
		}
		else
		{
			currentKeyFrame = 0;
		}
	}
	// reset timer
	glutTimerFunc(16, timer, 0);
}

void display(void)
{
	// clear buffer
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);


	// light source attributes
	float AmbientLight[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	float DiffuseLight[] = { 0.1f, 0.6f, 0.6f, 1.0f };
	float SpecularLight[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	float LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_AMBIENT, AmbientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, DiffuseLight);
	glLightfv(GL_LIGHT0, GL_SPECULAR, SpecularLight);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	// surface material attributes
	float kA[] = {0.3f, 1.0f, 1.0f, 1.0f };
	float kD[] = { 0.23f, 0.67f, 0.54f, 1.0f };
	float kS[] = { 0.13f, 0.33f, 0.52f, 1.0f };
	float kE[] = { 0.1f, 0.0f, 0.1f, 1.0f };
	float sE = 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT, kA);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, kD);
	glMaterialfv(GL_FRONT, GL_SPECULAR, kS);
	glMaterialfv(GL_FRONT, GL_EMISSION, kE);
	glMaterialf(GL_FRONT, GL_SHININESS, sE);

	// modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();


	// animation functions
	PelvisAnimation();
	LLegAnimation();
	LArmAnimation();
	RLegAnimation();
	RArmAnimation();
	SpineShoulderHeadAnimation();

	// light source
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	glutSwapBuffers();
}

// update viewport and projection matrix when the window is resized
void reshape(int w, int h)
{
	// viewport
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	// projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(70.0, (float)w / (float)h, 1.0, 30.0);
}

int main(int argc, char** argv) {
	// create GL window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Lab 2");

	// set callback functions
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutTimerFunc(16, timer, 0);

	// main loop
	glutMainLoop();

	return 0;
}


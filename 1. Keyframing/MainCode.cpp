/*
 * advancedTest.cpp
 *
 *  Created on: Sep 19, 2019
 *      Author: anway
 */

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <learnopengl/shader_m.h>
#include <learnopengl/camera.h>
#include <iostream>
#include <string.h>
#include <vector>
#include <GL/freeglut.h>

using namespace std;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;


float pathCoord[3];



//Set to change the type of animations
/*
 * 1 : Quaternion + Catmull-Rom Splines
 *
 * 2 : Quaternion + B-Splines
 *
 * 3 : fixedAngle Angles + Catmull-Rom Splines
 *
 * 4 : fixedAngle Angles + B-Splines
 *
 * */
int typeOfAnimation = 1;

// Number of Points for Spline
 int points = 0; //Current Point
int number = 6; //The total number of points

// time variables for generate Q(t)
float t = 0;

// The final M matrix for glLoadMatrixf()
float finalRotMat[4][4];

// The Catmul-Rom Spline M Matrix
float cSplineMat[16] = { -0.5f, 1.0f, -0.5f, 0.0f,
								1.5f, -2.5f, 0.0f, 1.0f,
								-1.5f, 2.0f, 0.5f, 0.0f,
								0.5f, -0.5f, 0.0f, 0.0f };

// The B Spline M Marix
float bSplineMat[16]= { -1.0f/6.0f, 3.0f/6.0f, -3.0f/6.0f, 1.0f/6.0f,
								3.0f/6.0f, -6.0f/6.0f, 0.0f/6.0f, 4.0f/6.0f,
								-3.0f/6.0f, 3.0f/6.0f, 3.0f/6.0f, 1.0f/6.0f,
								1.0f/6.0f, 0.0f/6.0f, 0.0f /6.0f, 0.0f/6.0f };

/*
 * The Key Positions for Quaternion Interpolation.
 */
float quatKeyFramePoints[6][7] = 			{{1, 0, 0, 0, -8, 8, -7 },
		  	  	  	  	  	  	  	  	  	{ 0, 1, 0, 0, -5, 1, -7 },
											{ 0, 0, 1, 0, -1, -3, -7 },
											{ 1, 0, 0, 1, 3, -1, -7 },
											{ 0, 1, 1, 0, 5, 5, -7 },
											{ 0, 1, 0, 0, 8, 8, -7 } };

/*
 * The Key Positions for Fixed Angles Interpolation.
 */
float fixedAngleKeyFramePoints[6][6] = { { 90, 0, 45, -8, 8, -7 },
											{ 70, 20, 65, -5, 1, -7 },
											{ 50, 40, 85, -1, -3, -7 },
											{ 30, 60, 105, 3, -1, -7 },
											{ 50, 40, 85, 6, 5, -7 },
											{ 70, 20, 65, 8, 8, -7 }};

/*
 * Blending Function
 * This Function gives the interpolated value of the positional and orientation coordinates
 * T,M and G values are blended to return the Q(t) value
 */
float BlendingFunction(float T[4], float m[16], float g[4])
{
	float b[4];
	b[0] = T[0] * m[0] + T[1] * m[1] + T[2] * m[2] + T[3] * m[3];

	b[1] = T[0] * m[4] + T[1] * m[5] + T[2] * m[6] + T[3] * m[7];

	b[2] = T[0] * m[8] + T[1] * m[9] + T[2] * m[10] + T[3] * m[11];

	b[3] = T[0] * m[12] + T[1] * m[13] + T[2] * m[14] + T[3] * m[15];

	float qT = b[0] * g[0] + b[1] * g[1] + b[2] * g[2] + b[3] * g[3];

	return qT;
}

/*
 * The Obtained interpolated coordinated must be normalized
 * This is to get all values between 0 and 1
 */
void Normalize(float temp[7])
{
	float sq;
	sq = temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2] + temp[3]*temp[3];

	if (sq != 0)
	{
		sq = sqrt(sq);
		temp[0] = (float)temp[0]/(float)sq;
		temp[1] = (float)temp[1]/(float)sq;
		temp[2] = (float)temp[2]/(float)sq;
		temp[3] = (float)temp[3]/(float)sq;
	}
}

/*
 *  Quaternion Rotation Matrix is generated with with the interpolated coordinates
 *  Final rotation matrix is created that is to be loaded to the rendering pipeline
 */
void QRotation(float temp[7], float R[4][4])
{
	float w = temp[0];
	float x = temp[1];
	float y = temp[2];
	float z = temp[3];

	R[0][0] = 1.0f - 2.0f*y*y - 2.0f*z*z;
	R[0][1] = 2.0f*x*y + 2.0f*w*z;
	R[0][2] = 2.0f*x*z - 2.0f*w*y;
	R[0][3] = 0.0f;
	R[1][0] = 2.0f*x*y - 2.0f*w*z;
	R[1][1] = 1.0f - 2.0f*x*x - 2.0f*z*z;
	R[1][2] = 2.0f*y*z + 2.0f*w*x;
	R[1][3] = 0.0f;
	R[2][0] = 2.0f*x*z + 2.0f*w*y;
	R[2][1] = 2.0f*y*z - 2.0f*w*x;
	R[2][2] = 1.0f - 2.0f*x*x - 2.0f*y*y;
	R[2][3] = 0.0f;
	R[3][0] = temp[4];
	R[3][1] = temp[5];
	R[3][2] = temp[6];
	R[3][3] = 1.0f;
}

/*
 *  Generate Quaternion with Given Fixed Angle/
 */

void FixedAngleToQuaternion(float temp[7])
{
	float a = temp[0] / 2;
	float b = temp[1] / 2;
	float c = temp[2] / 2;

	//The X,Y,Z positions
	temp[6] = temp[5];
	temp[5] = temp[4];
	temp[4] = temp[3];

	//The w,x,y,z quaternion rotation
	temp[0] = cos(c)*cos(b)*cos(c) + sin(c)*sin(b)*sin(a);
	temp[1] = sin(c)*cos(b)*cos(c) - cos(c)*sin(b)*sin(a);
	temp[2] = cos(c)*sin(b)*cos(c) + sin(c)*cos(b)*sin(a);
	temp[3] = cos(c)*cos(b)*sin(c) - sin(c)*sin(b)*cos(a);
}

// Generate interpolation With coordinate data and spline data forFixed Angles
void FixedAngleInterpolate(float fixedAnglePoints[6][6], float splineMat[16])
{
	//T Matrix
	float tMatrix[4];
	tMatrix[0]=t*t*t;
	tMatrix[1]=t*t;
	tMatrix[2]=t;
	tMatrix[3]=1;

	float temp[7];

	for (int i=0; i<7; i++)
	{
		//Geometric data of the 4 closest key frames
		float gMatrix[4];
		gMatrix[0] = fixedAnglePoints[points][i];
		gMatrix[1] = fixedAnglePoints[(points + 1)][i];
		gMatrix[2] = fixedAnglePoints[(points + 2)][i];
		gMatrix[3] = fixedAnglePoints[(points + 3)][i];

		//Blend to get interpolated coordinates
		temp[i] = BlendingFunction(tMatrix, splineMat, gMatrix);
	}

	FixedAngleToQuaternion(temp);
	Normalize(temp);
	QRotation(temp,finalRotMat);
}


// Generate interpolation With coordinate data and spline data for Quaternions
void QuaternionInterpolate(float quatPoints[6][7], float splineMat[16])
{
	//T Matrix
	float tMatrix[4];
	tMatrix[0]=t*t*t;
	tMatrix[1]=t*t;
	tMatrix[2]=t;
	tMatrix[3]=1;

	float temp[7];

	for (int i=0; i<7; i++)
	{
		//Geometric data of the 4 closest key frames
		float gMatrix[4] = { quatPoints[points][i],
							quatPoints[points + 1][i],
							quatPoints[points + 2][i],
							quatPoints[points + 3][i]};

		//Blend to get interpolated coordinates
		temp[i] = BlendingFunction(tMatrix, splineMat, gMatrix);
	}
	Normalize(temp);
	QRotation(temp,finalRotMat);
}


int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);


    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Animation Lab1", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    //glfwSetCursorPosCallback(window, mouse_callback);
    //glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);

    // build and compile our shader zprogram
    // ------------------------------------
    Shader ourShader("vertShader.vs", "fragShader.fs");

    // set up vertex data (and buffer(s)) and configure vertex attributes
    // ------------------------------------------------------------------
    float vertices[] = {
        -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,
         0.5f, -0.5f, -0.5f,  1.0f, 0.0f,
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,

        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
         0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
        -0.5f,  0.5f,  0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,

        -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
        -0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
        -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
         0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
         0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
         0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
         0.5f, -0.5f, -0.5f,  1.0f, 1.0f,
         0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
         0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,

        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
        -0.5f,  0.5f,  0.5f,  0.0f, 0.0f,
        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f
    };

    unsigned int VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // texture coord attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);


    // load and create a texture
    // -------------------------
    unsigned int texture1, texture2;
    // texture 1
    // ---------
    glGenTextures(1, &texture1);
    glBindTexture(GL_TEXTURE_2D, texture1);
    // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // load image, create texture and generate mipmaps
    int width, height, nrChannels;
    stbi_set_flip_vertically_on_load(true); // tell stb_image.h to flip loaded texture's on the y-axis.
    unsigned char *data = stbi_load("container.jpg", &width, &height, &nrChannels, 0);
    if (data)
    {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    else
    {
        std::cout << "Failed to load texture" << std::endl;
    }
    stbi_image_free(data);
    // texture 2
    // ---------
    glGenTextures(1, &texture2);
    glBindTexture(GL_TEXTURE_2D, texture2);
    // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // load image, create texture and generate mipmaps
    data = stbi_load("awesomeface.png", &width, &height, &nrChannels, 0);
    if (data)
    {
        // note that the awesomeface.png has transparency and thus an alpha channel, so make sure to tell OpenGL the data type is of GL_RGBA
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    else
    {
        std::cout << "Failed to load texture" << std::endl;
    }
    stbi_image_free(data);

    // tell opengl for each sampler to which texture unit it belongs to (only has to be done once)
    // -------------------------------------------------------------------------------------------
    ourShader.use();
    ourShader.setInt("texture1", 0);
    ourShader.setInt("texture2", 1);

    float temp[4][4];
    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        // per-frame time logic
        // --------------------
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
    	t = t + 0.01;
    	if (t >= 1)
    	{
    		t = 0;
    		if (points < number - 4)
    		{
    			points++;
    		}
    		else
    		{
    			points = 0;
    		}
    	}

        // input
        // -----
        processInput(window);

        // render
        // ------
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // bind textures on corresponding texture units
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture1);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, texture2);

        // activate shader
        ourShader.use();

        // pass projection matrix to shader (note that in this case it could change every frame)
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);


        ourShader.setMat4("projection", projection);

        // camera/view transformation
        glm::mat4 view = camera.GetViewMatrix();
        ourShader.setMat4("view", view);


        // render boxes
        glBindVertexArray(VAO);
        // calculate the model matrix for each object and pass it to shader before drawing
        glm::mat4 model = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first


	//Switch case to decide the type of animation function called
    	switch(typeOfAnimation)
    	{
    		case 1:
    		{
    			QuaternionInterpolate(quatKeyFramePoints,cSplineMat);
    			break;
    		}
    		case 2:
    		{
    			QuaternionInterpolate(quatKeyFramePoints,bSplineMat);
    			break;
    		}
    		case 3:
    		{
    			FixedAngleInterpolate(fixedAngleKeyFramePoints, cSplineMat);
    			break;
    		}
    		case 4:
    		{
    			FixedAngleInterpolate(fixedAngleKeyFramePoints, bSplineMat);
    			break;
    		}
    		default:
    		{
    			cout<<"\nEnter Valid Animation Coordination";
    			break;
    		}
    	}
    	
    	//After obtaining the final rotation matrix, we need to assign it as the model matrix
    	for(int i=0;i<4;i++)
    	{
    		for(int j=0;j<4;j++)
    		{
    			int k = i*4+j;
    			model[i][j] = finalRotMat[i][j];
    		}
    	}

	//Model Matrix assigned in the shader
        ourShader.setMat4("model", model);

	//Draw the cube.
        glDrawArrays(GL_TRIANGLES, 0, 36);

        glfwSwapBuffers(window);
        glfwPollEvents();

    }

    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}



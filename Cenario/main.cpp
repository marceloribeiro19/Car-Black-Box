#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#define _USE_MATH_DEFINES
#include <math.h>
#include <boost/asio.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <thread>
#include <string>
#include <sstream>

using namespace std;

struct point
{
	float x, y, z;

	point(float x, float y, float z) : x(x), y(y), z(z){};
};

typedef struct imuData{
	float pitch;
	float roll;
    float yaw;
}ASM330_LHB;

float x = 0.0f, y = 0.0f, z = 0.0f;
vector<point> line;
static float angleX = 0;
static float angleY = 0;
static float angleZ = 0;

ASM330_LHB imu;

/*
 * Changes scenario scale based on the window size 
 */
void changeSize(int w, int h)
{
	// prevent a divide by zero, when window is too short
	// (you can not make a window with zero width).
	if (h == 0)
		h = 1;
	// compute window's aspect ratio
	float ratio = w * 1.0f / h;

	// set the projection matrix as current
	glMatrixMode(GL_PROJECTION);
	// Load the identity matrix
	glLoadIdentity();
	// set the perspective
	gluPerspective(45.0f, ratio, 1.0f, 1000.0f);
	// return to the model view matrix mode
	glMatrixMode(GL_MODELVIEW);

	// et the viewport to be the entire window
	glViewport(0, 0, w, h);
}

/*
 * Serial Functions 
 */
static void extractFloat(string serialData){
	// Create a string stream to parse the data
    istringstream iss(serialData);

    // Iterate through each word in the string
    string word;
    while (iss >> word) {
        if (word == "Roll->") {
            // Extract the next word as the roll angle
            iss >> word;
            imu.roll = stof(word);
        }
        else if (word == "Pitch->") {
            // Extract the next word as the pitch angle
            iss >> word;
            imu.pitch = stof(word); 
        }
       else if(word == "Yaw->"){
            // Extract the next word as the Yaw angle
            iss >> word;
            imu.yaw = stof(word); 
        }
    }
}
void serialCOM(){
	// Set up serial port parameters
    boost::asio::io_service io;
    boost::asio::serial_port port(io);
    port.open("/dev/ttyUSB0");  // Specify your serial port device here

    port.set_option(boost::asio::serial_port_base::baud_rate(115200));
    port.set_option(boost::asio::serial_port_base::character_size(8));
    port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    // Read data from serial port
    try {
        while (true) {
            boost::asio::streambuf buf;
            boost::asio::read_until(port, buf, '\n');  // Read until newline
            istream is(&buf);
            string line;
            getline(is, line);
			extractFloat(line);
			
			// Print the extracted roll and pitch angles
			cout << "Roll : " << imu.roll << " degrees" << endl;
			cout << "Pitch: " << imu.pitch  << " degrees" << endl;
            cout << "Yaw  : " << imu.yaw  << " degrees" << endl;
        }
    } catch (exception& e) {
        cerr << "Exception: " << e.what() << endl;
    }
}
void serialThread() {
    /* Thread to get IMU data parallelly to the scene rendering*/
	while (true) {
        serialCOM();
    }
}

/*
 * Init Scenario Functions
 */
static void drawScene(void){
	/* Drawing instructions */ 
	
	/*glColor3f(1.0f, 0.0f, 0.0f); 	//axis color

	/* Draws a square in the floor for perspective */
   /* glColor3f(0.7f, 0.7f, 0.7f); // Cor cinza claro
    glBegin(GL_QUADS);
    glVertex3f(-20.0f, 0.0f, -20.0f); // Canto inferior esquerdo
    glVertex3f(-20.0f, 0.0f, 20.0f); // Canto superior esquerdo
    glVertex3f(20.0f, 0.0f, 20.0f); // Canto superior direito
    glVertex3f(20.0f, 0.0f, -20.0f); // Canto inferior direito
    glEnd();
*/

 // Eixo X - Vermelho
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(450.0f, 0.0f, 0.0f);
    glEnd();

    // Eixo Y - Verde
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 450.0f, 0.0f);
    glEnd();

    // Eixo Z - Azul
    glColor3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 450.0f);
    glEnd();

    glColor3f(0.7f, 0.7f, 0.7f);
    glBegin(GL_LINES);
	for (int i = 0; i < line.size() - 1; i++)
	{
		glVertex3f(line[i].x, line[i].y, line[i].z);
		glVertex3f(line[i + 1].x, line[i + 1].y, line[i + 1].z);
	}

	glEnd();

}
static void cameraPlacement(void){
	// Defines the camera position, looking at the cube
    float cubePos[3] = { line.back().x, line.back().y, line.back().z };
    float cameraPos[3] = { cubePos[0] + 40.0f, cubePos[1] + 0.0f, cubePos[2] + 0.0f };
    gluLookAt(cameraPos[0], cameraPos[1], cameraPos[2],
              cubePos[0], cubePos[1], cubePos[2],
              0.0f, 0.0f, 1.0f);
              
}
static void initScene(void){
	// clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// set camera
	glLoadIdentity();
	cameraPlacement();
	drawScene();
}

/*
 * Cube Functions
 */
static void drawCubeFace(float x1, float y1, float z1,
                          float x2, float y2, float z2,
                          float x3, float y3, float z3,
                          float x4, float y4, float z4,
                          float scale) {
    // Aplica a escala aos vértices
    x1 *= scale; y1 *= scale; z1 *= scale;
    x2 *= scale; y2 *= scale; z2 *= scale;
    x3 *= scale; y3 *= scale; z3 *= scale;
    x4 *= scale; y4 *= scale; z4 *= scale;

    glBegin(GL_QUADS);
    glVertex3f(x1, y1, z1);
    glVertex3f(x2, y2, z2);
    glVertex3f(x3, y3, z3);
    glVertex3f(x4, y4, z4);
    glEnd();
}

static void drawCustomCube(float width, float height, float depth, float scale) {
    // Define as coordenadas dos vértices para cada face do cubo
    float halfWidth = width / 2.0f;
    float halfHeight = height / 2.0f;
    float halfDepth = depth / 2.0f;

    // Front face
    glColor3f(1.0f, 0.0f, 0.0f);//RED
    drawCubeFace(-halfWidth, -halfHeight, halfDepth,halfWidth, -halfHeight, halfDepth,halfWidth, halfHeight, halfDepth,-halfWidth, halfHeight, halfDepth,scale);

    // Back face
    glColor3f(0.0f, 1.0f, 0.0f);//Green
    drawCubeFace(-halfWidth, -halfHeight, -halfDepth,-halfWidth, halfHeight, -halfDepth,halfWidth, halfHeight, -halfDepth,halfWidth, -halfHeight, -halfDepth,scale);

    // Top face
    glColor3f(0.0f, 0.0f, 1.0f);//Blue
    drawCubeFace(-halfWidth, halfHeight, -halfDepth,-halfWidth, halfHeight, halfDepth,halfWidth, halfHeight, halfDepth,halfWidth, halfHeight, -halfDepth,scale);

    // Bottom face
    glColor3f(1.0f, 1.0f, 0.0f);//Yellow
    drawCubeFace(-halfWidth, -halfHeight, -halfDepth,halfWidth, -halfHeight, -halfDepth,halfWidth, -halfHeight, halfDepth,-halfWidth, -halfHeight, halfDepth,scale);

    // Right face
    glColor3f(1.0f, 0.0f, 1.0f);//Purple
    drawCubeFace(halfWidth, -halfHeight, -halfDepth,halfWidth, halfHeight, -halfDepth,halfWidth, halfHeight, halfDepth,halfWidth, -halfHeight, halfDepth,scale);

    // Left face
    glColor3f(1.0f, 0.5f, 0.0f);//Orange
    drawCubeFace(-halfWidth, -halfHeight, -halfDepth,-halfWidth, -halfHeight, halfDepth,-halfWidth, halfHeight, halfDepth,-halfWidth, halfHeight, -halfDepth,scale);
}

static void rotateCube(float angleX, float angleY, float angleZ){
	// Convert degrees to radians
    float radiansX = angleX * M_PI / 180.0f;
    float radiansY = angleY * M_PI / 180.0f;

	glTranslatef(line.back().x, line.back().y, line.back().z );
	glRotatef(angleX, 1.0f, 0.0f, 0.0f); // X Rotation
	glRotatef(angleY, 0.0f, 1.0f, 0.0f); // Y Rotation
	glRotatef(angleZ, 0.0f, 0.0f, 1.0f); // Z Rotation
	drawCustomCube(1,1,1,10);
}

/*
 * Render Full Scenario
 */
void renderScene(void)
{
	initScene();

	rotateCube(imu.roll,imu.pitch,imu.yaw);

	/* Line Movement */
	line.push_back(point(x, y, z));
  
    //x += 1.0f;
	//x += 0.1f;
    //y += 0.1f;
	//z += 0.1f;

	// End of frame
	glutSwapBuffers();
}

void printInfo()
{
	printf("Vendor: %s\n", glGetString(GL_VENDOR));
	printf("Renderer: %s\n", glGetString(GL_RENDERER));
	printf("Version: %s\n", glGetString(GL_VERSION));
}

int main(int argc, char **argv)
{
	// put GLUT init here
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(1000, 1000);
	glutCreateWindow("IMU IN REAL-TIME");

	// put callback registry here
	glutDisplayFunc(renderScene);
	glutReshapeFunc(changeSize);
	glutIdleFunc(renderScene);

	// some OpenGL settings
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	line.push_back(point(0.0f, 0.0f, 0.0f));


	thread serial(serialThread);

	// enter GLUTs main cycle
	glutMainLoop();

	serial.join();
	
	return 1;
}

// Author: Daren Cheng
// Class:  CS 4392
// Date:   11/28/2015

// Desc: 
// This program demonstrates basic collision detection via SWIFT
// it is based off example.c from the SWIFT API

//include dependancies
#include <AntTweakBar.h>
#include <GL/freeglut.h>
#include <windows.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <SWIFT.h>

//include GLM extensions, someday!
//#include <glm/vec3.hpp>
//#include <glm/vec4.hpp>
//#include <glm/mat4x4.hpp>
//#include <glm/gtc/matrix_transform.hpp>

//include local header files
#include <Domain/Scene.h>
#include <Utils/Functions.h>

//function prototypes
void display();
void reshape(int width, int height);
void TW_CALL rotate(void *clientData);
void TW_CALL invert(void *clientData);
void TW_CALL zero(void *clientData);
void TW_CALL clear(void *clientData);

//variables that determine rendering setings
TwBar *bar;                                           //pointer to tweakbar
float ambientColor[] = { 0.2f, 0.2f, 0.2f, 1.0f };    //ambient light color
float sunColor[] = { 0.8f, 0.8f, 0.8f, 1.0f };        //sun light color
float sunPosition[] = { 2.5f, 2.5f, 2.5f, 0.0f };     //position of sunlight
float moonColor[] = { 0.2f, 0.2f, 0.4f, 1.0f };       //moon light color
float moonPosition[] = { -2.5f, 1.5f, -1.5f, 0.0f };  //position of moonlight
float lineColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };       //color of object outlines 
bool wireframe = false;                               //render wireframes

//variables defined globally so they aren't redefined each display call
Scene scene;                 //scene consisting of rigid bodies
double initial_time;         //initial time of program starting
double current_time;         //current time from system clock
double previous_time;        //time of previous update
double elapsed_time;         //time elapsed between physics updates
double update_fps;           //updates per second
double contact_restitution;  //restitution for contact handling, ranges from -1 to 0
bool collision = true;       //indicates if there are any collisions
bool contact = true;         //indicates if there are any contacts
int obj_id = 0;              //id of object to apply impulse too
int scene_id = 1;            //id of scene to reset to

//impulse parameters
double location[3] = { 0, 0, 0 };
double impulse[3] = { 0, 0, 0 };

//object filenames
static const char* floor_box = "PhysicsEngine/Objects/floor.tri";
static const char* cube = "PhysicsEngine/Objects/cube.tri";
static const char* icosa = "PhysicsEngine/Objects/icosa.tri";
static const char* sphere = "PhysicsEngine/Objects/sphere.tri";
static const char* cylinder = "PhysicsEngine/Objects/cylinder.tri";
static const char* UVsphere = "PhysicsEngine/Objects/UVsphere.tri";
static const char* sphereHD = "PhysicsEngine/Objects/sphereHD.tri";

//defined constants
double IDENTITY[9] =
{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
};
double ORIGIN[4] =
{
    0.0, 0.0, 0.0,
    0.0
};

//lookat and viewport parameters
int width = 640;
int height = 480;
double eye_x = 0.0;
double eye_y = 0.0;
double eye_z = 0.0;
double center_x = 0.0;
double center_y = 0.0;
double center_z = 0.0;

using namespace std;

class Boxscene : public Scene {
public:
    Boxscene::Boxscene() {
        this->gravity = 0;

        //offset of cubes
        double offset[4] = { 0, 1.01, 0, 0 };
        double p_velocity[3] = { 0 };
        double l_velocity[3] = { 0 };
        double density = 1.0;
        double restitution = 1.0;

        //add cube at origin
        this->add_body(cube, IDENTITY, offset, false, 
            density, p_velocity, l_velocity, restitution);

        //add an offset cube
        offset[2] = 1.2 * 2;
        this->copy_body(0, IDENTITY, offset, false, 
            p_velocity, l_velocity, restitution);

        //add another offset cube
        offset[2] = 4.8 * 2;
        this->copy_body(0, IDENTITY, offset, false,
            p_velocity, l_velocity, restitution);

        //add floor
        offset[2] = 0.0;
        offset[1] = -5;
        p_velocity[1] = 0;
        l_velocity[1] = 0;
        float color[4] = { 0.5, 0.5, 0.5, 1.0 };
        restitution = 1;
        //this->add_body(floor_box, color, IDENTITY, offset, true, 
        //    density, p_velocity, l_velocity, restitution);

        //activate the swift scene
        this->activate();
    }
};

class SingleBoxscene : public Scene {
public:
    SingleBoxscene::SingleBoxscene() {
        this->gravity = 0;

        //offset of cubes
        double offset[4] = { 0, 1.01, 0, 0 };
        double p_velocity[3] = { 0 };
        double l_velocity[3] = { 0 };
        double density = 1.0;
        double restitution = 1.0;

        //add cube at origin
        this->add_body(cube, IDENTITY, offset, false,
            density, p_velocity, l_velocity, restitution);

        //activate the swift scene
        this->activate();
    }
};

class BallScene : public Scene {
public:
    BallScene::BallScene() {

        //offset of cubes
        double offset[4] = { 0, 4.05, 0, 0 };
        double p_velocity[3] = { 0 };
        double l_velocity[3] = { 0 };
        double density = 0.5;
        double restitution = 1;

        //add cube at origin
        offset[2] = 1.2 * -2;
        this->add_body(UVsphere, IDENTITY, offset, false,
            density, p_velocity, l_velocity, restitution);

        //add an offset cube
        offset[2] = 0;
        restitution = 0.8;
        this->copy_body(0, IDENTITY, offset, false,
            p_velocity, l_velocity, restitution);

        //add another offset cube
        offset[2] = 1.2 * 2;
        restitution = 0;
        this->copy_body(0, IDENTITY, offset, false,
            p_velocity, l_velocity, restitution);

        //add another offset cube
        offset[2] = 4.8 * 2;
        this->copy_body(0, IDENTITY, offset, true,
            p_velocity, l_velocity, restitution);

        //add floor
        offset[2] = 0.0;
        offset[1] = -5;
        p_velocity[1] = 0;
        l_velocity[1] = 0;
        float color[4] = { 0.5, 0.5, 0.5, 0.5 };
        restitution = 1;
        this->add_body(floor_box, color, IDENTITY, offset, true,
            density, p_velocity, l_velocity, restitution);

        //activate the swift scene
        this->activate();
    }
};

class MultiScene : public Scene {
public:
    MultiScene::MultiScene() {
        this->gravity = -.98;
        
        //offset of cubes
        double offset[4] = { 0, 0.0, 0, 0 };
        double p_velocity[3] = { 0 };
        double l_velocity[3] = { 0 };
        double density = 1;
        double restitution = 1;

        //add cube at origin
        offset[1] = 5;
        p_velocity[1] = -5;
        this->add_body(sphere, IDENTITY, offset, false,
            density, p_velocity, l_velocity, restitution);

        //add various offsets
        offset[2] = 0;
        p_velocity[1] = 0;
        bool switcher = true;
        for (double i = -2; i <= 2; i++) {
            for (double j = -2; j <= 2; j++) {
                for (double k = 0; k < 3; k++) {
                    offset[0] = i * 1.2;
                    offset[1] = k  + 0.5;
                    offset[2] = j * 1.2;
                    if (switcher) {
                        this->copy_body(0, IDENTITY, offset, false,
                            p_velocity, l_velocity, restitution);
                        switcher = false;
                    }
                    else {
                        switcher = true;
                    }
                }
            }
        }

        //add floor
        offset[3] = 0.0;
        offset[2] = 0.0;
        offset[0] = 0.0;
        offset[1] = -5;
        p_velocity[1] = 0;
        l_velocity[1] = 0;
        float color[4] = { 0.5, 0.5, 0.5, 1.0 };
        restitution = 1;
        this->add_body(floor_box, color, IDENTITY, offset, true,
            density, p_velocity, l_velocity, restitution);

        //activate the swift scene
        this->activate();
    }
};

class MultiSceneLight : public Scene {
public:
    MultiSceneLight::MultiSceneLight() {
        this->gravity = -1.98;

        //offset of cubes
        double offset[4] = { 0, 0.0, 0, 0 };
        double p_velocity[3] = { 0 };
        double l_velocity[3] = { 0 };
        double density = 2.5;
        double restitution = 1;

        //add cube at origin
        offset[1] = 5;
        p_velocity[1] = -5;
        this->add_body(sphere, IDENTITY, offset, false,
            density, p_velocity, l_velocity, restitution);

        //add various offsets
        offset[2] = 0;
        p_velocity[1] = 0;
        bool switcher = true;
        for (double i = -1; i <= 1; i++) {
            for (double j = -1; j <= 1; j++) {
                for (double k = 0; k < 2; k++) {
                    offset[0] = i * 1.2;
                    offset[1] = k + 0.5;
                    offset[2] = j * 1.2;
                    if (switcher) {
                        this->copy_body(0, IDENTITY, offset, false,
                            p_velocity, l_velocity, restitution);
                        switcher = false;
                    }
                    else {
                        switcher = true;
                    }
                }
            }
        }

        //add floor
        offset[3] = 0.0;
        offset[2] = 0.0;
        offset[0] = 0.0;
        offset[1] = -5;
        p_velocity[1] = 0;
        l_velocity[1] = 0;
        float color[4] = { 0.5, 0.5, 0.5, 1.0 };
        restitution = 1;
        this->add_body(floor_box, color, IDENTITY, offset, true,
            density, p_velocity, l_velocity, restitution);

        //activate the swift scene
        this->activate();
    }
};

class PillarScene : public Scene {
public:
    PillarScene::PillarScene() {
        this->gravity = 0;

        //offset of cubes
        double offset[4] = { 0, 1.01, 0, 0 };
        double p_velocity[3] = { 0 };
        double l_velocity[3] = { 0 };
        double density = 1.0;
        double restitution = 1.0;

        //add cylinder at origin
        density = 0.1;
        this->add_body(cylinder, IDENTITY, offset, false,
            density, p_velocity, l_velocity, restitution);

        //fire cubes at cylinder
        density = 2.0;
        offset[0] = 7.5;
        offset[2] = 15.0;
        p_velocity[2] = -10.0;
        this->add_body(icosa, IDENTITY, offset, false,
            density, p_velocity, l_velocity, restitution);
        offset[0] = -7.5;
        offset[2] = -15.0;
        p_velocity[2] = 10.0;
        this->copy_body(1, IDENTITY, offset, false,
            p_velocity, l_velocity, restitution);

        //activate the swift scene
        this->activate();
    }
};

class CollideScene : public Scene {
public:
    CollideScene::CollideScene() {
        this->gravity = -4.8;

        //offset of cubes
        double offset[4] = { 0, 2.51, 0, 0 };
        double p_velocity[3] = { 0 };
        double l_velocity[3] = { 0 };
        double density = 0.1;
        double restitution = 1.0;
        double y_jump;
        int chooser;

        //randomly generate positive jump in y
        y_jump = rand() % 100 / 20.0;

        //randomly generate starting locations and velocities
        offset[0] = rand() % 300 / 20.0 - 7.5;
        offset[2] = rand() % 300 / 20.0 - 7.5;
        p_velocity[0] = -offset[0];
        p_velocity[1] = 2.5 + y_jump;
        p_velocity[2] = -offset[2];
        l_velocity[0] = rand() % 200 / 20.0 - 5.0;
        l_velocity[1] = rand() % 200 / 20.0 - 5.0;
        l_velocity[2] = rand() % 200 / 20.0 - 5.0;

        //randomly choose what objects to collide
        chooser = rand() % 3 + 1;
        switch (chooser) {
        case 1:
            this->add_body(UVsphere, IDENTITY, offset, false,
                density, p_velocity, l_velocity, restitution);
            break;
        case 2:
            this->add_body(cube, IDENTITY, offset, false,
                density, p_velocity, l_velocity, restitution);
            break;
        case 3:
            this->add_body(icosa, IDENTITY, offset, false,
                density, p_velocity, l_velocity, restitution);
            break;
        default:
            this->add_body(icosa, IDENTITY, offset, false,
                density, p_velocity, l_velocity, restitution);
            break;
        }

        //do the same for the second object
        //randomly generate starting locations and velocities
        offset[0] = rand() % 300 / 20.0 - 7.5;
        offset[2] = rand() % 300 / 20.0 - 7.5;
        p_velocity[0] = -offset[0];
        p_velocity[1] = 2.5 + y_jump;
        p_velocity[2] = -offset[2];
        l_velocity[0] = rand() % 200 / 20.0 - 5.0;
        l_velocity[1] = rand() % 200 / 20.0 - 5.0;
        l_velocity[2] = rand() % 200 / 20.0 - 5.0;

        //randomly choose what objects to collide
        chooser = rand() % 3 + 1;
        switch (chooser) {
        case 1:
            this->add_body(UVsphere, IDENTITY, offset, false,
                density, p_velocity, l_velocity, restitution);
            break;
        case 2:
            this->add_body(cube, IDENTITY, offset, false,
                density, p_velocity, l_velocity, restitution);
            break;
        case 3:
            this->add_body(icosa, IDENTITY, offset, false,
                density, p_velocity, l_velocity, restitution);
            break;
        default:
            this->add_body(icosa, IDENTITY, offset, false,
                density, p_velocity, l_velocity, restitution);
            break;
        }

        //add floor
        offset[3] = 0.0;
        offset[2] = 0.0;
        offset[0] = 0.0;
        offset[1] = -5;
        p_velocity[0] = 0;
        p_velocity[1] = 0;
        p_velocity[2] = 0;
        l_velocity[0] = 0;
        l_velocity[1] = 0;
        l_velocity[2] = 0;
        float color[4] = { 0.5, 0.5, 0.5, 1.0 };
        restitution = 1;
        this->add_body(floor_box, color, IDENTITY, offset, true,
            density, p_velocity, l_velocity, restitution);

        //activate the swift scene
        this->activate();
    }
};

class StackingScene : public Scene {
public:
    StackingScene::StackingScene() {

        //offset of cubes
        double offset[4] = { 0, 1.02, 0, 0 };
        double p_velocity[3] = { 0 };
        double l_velocity[3] = { 0 };
        double density = 1.0;
        double restitution = 0.0;

        //add cube at origin
        this->add_body(cube, IDENTITY, offset, false,
            density, p_velocity, l_velocity, restitution);

        //add stack of cubes
        for (int i = 0; i < 5; i++) {
            offset[1] += 2.02;
            this->copy_body(0, IDENTITY, offset, false,
                p_velocity, l_velocity, restitution);
        }

        //add floor
        offset[3] = 0.0;
        offset[2] = 0.0;
        offset[0] = 0.0;
        offset[1] = -5;
        p_velocity[0] = 0;
        p_velocity[1] = 0;
        p_velocity[2] = 0;
        l_velocity[0] = 0;
        l_velocity[1] = 0;
        l_velocity[2] = 0;
        float color[4] = { 0.5, 0.5, 0.5, 1.0 };
        restitution = 0;
        this->add_body(floor_box, color, IDENTITY, offset, true,
            density, p_velocity, l_velocity, restitution);

        //activate the swift scene
        this->activate();
    }
};

int main(int argc, char **argv)
{
    //set camera position as specified
    eye_x = stod(argv[1]);
    eye_y = stod(argv[2]);
    eye_z = stod(argv[3]);
    center_x = stod(argv[4]);
    center_y = stod(argv[5]);
    center_z = stod(argv[6]);

    //set update fps as specified
    update_fps = stod(argv[7]);

    //create a boxscene
    scene = BallScene();
    scene.draw();

    //initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(640, 480);
    glutCreateWindow("AnimParser.cpp");
    glutCreateMenu(NULL);

    //get GLUT callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);

    //initialize AntTweakBar
    TwInit(TW_OPENGL, NULL);

    //control redirect all mouse and keyboard events to AntTweakBar
    //glutIgnoreKeyRepeat(true);
    glutKeyboardFunc((GLUTkeyboardfun)TwEventKeyboardGLUT);
    glutMouseFunc((GLUTmousebuttonfun)TwEventMouseButtonGLUT);
    glutMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);
    glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);
    TwGLUTModifiersFunc(glutGetModifiers);

    //create a tweak bar
    bar = TwNewBar("Translations");

    //add intersection indicator
    TwAddVarRW(bar, "Update FPS", TW_TYPE_DOUBLE, &update_fps,
        " min=1 max=144 step=1");
    TwAddVarRW(bar, "Wireframe", TW_TYPE_BOOLCPP, &wireframe, NULL);
    TwAddVarRW(bar, "Scene ID", TW_TYPE_INT8, &scene_id, NULL);
    TwAddButton(bar, "Start Scene", rotate, NULL, NULL);
    TwAddSeparator(bar, NULL, NULL);

    //add object controls
    TwAddVarRW(bar, "Object ID", TW_TYPE_INT8, &obj_id, NULL);
    TwAddVarRW(bar, "Location X", TW_TYPE_DOUBLE, &location[0], " min=-1 max=1 step=0.1 ");
    TwAddVarRW(bar, "Location Y", TW_TYPE_DOUBLE, &location[1], " min=-1 max=1 step=0.1 ");
    TwAddVarRW(bar, "Location Z", TW_TYPE_DOUBLE, &location[2], " min=-1 max=1 step=0.1 ");
    TwAddVarRW(bar, "Impulse X", TW_TYPE_DOUBLE, &impulse[0], " min=-10 max=10 step=0.25 ");
    TwAddVarRW(bar, "Impulse Y", TW_TYPE_DOUBLE, &impulse[1], " min=-10 max=10 step=0.25 ");
    TwAddVarRW(bar, "Impulse Z", TW_TYPE_DOUBLE, &impulse[2], " min=-10 max=10 step=0.25 ");
    TwAddButton(bar, "Apply Impulse", zero, NULL, "key=space");
    TwAddButton(bar, "Clear", clear, NULL, NULL);
    TwAddButton(bar, "Invert", invert, NULL, NULL);
    TwAddSeparator(bar, NULL, NULL);

    //add camera motions
    TwAddVarRW(bar, "Camera X", TW_TYPE_DOUBLE, &eye_x,
        " min=-30 max=30 step=0.5 keyIncr=d keyDecr=a ");
    TwAddVarRW(bar, "Camera Y", TW_TYPE_DOUBLE, &eye_y,
        " min=-30 max=30 step=0.5 keyIncr=e keyDecr=q ");
    TwAddVarRW(bar, "Camera Z", TW_TYPE_DOUBLE, &eye_z,
        " min=-30 max=30 step=0.5 keyIncr=s keyDecr=w ");
    TwAddSeparator(bar, NULL, NULL);
    
    //add light color controllers
    //TwAddVarRW(bar, "Sunlight Color", TW_TYPE_COLOR4F, &sunColor, NULL);
    //TwAddVarRW(bar, "MoonLight Color", TW_TYPE_COLOR4F, &moonColor, NULL);
    //TwAddVarRW(bar, "SunLight Position", TW_TYPE_DIR3F, &sunPosition, NULL);
    //TwAddVarRW(bar, "MoonLight Position", TW_TYPE_DIR3F, &moonPosition, NULL);

    //initialize time
    initial_time = (double)GetTickCount64() / 1000.0;
    current_time = (double)GetTickCount64() / 1000.0 - initial_time;
    previous_time = (double)GetTickCount64() / 1000.0 - initial_time;

    //call the GLUT main loop
    glutMainLoop();

    cin.get();
    return 0;
}

//callback function passed to glutDisplayFunc
void display() {
    //clear frame buffer to avoid rendering issues
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //rendering settings
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_NORMALIZE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glShadeModel(GL_SMOOTH);
    if (wireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }
    else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    //set camera angle and position, along with viewport
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (double)width / height, 1, 100);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eye_x, eye_y, eye_z, center_x, center_y, center_z, 0, 1, 0);

    //draw WCS axis
    drawWCSAxis();

    //Enable lighting and create a light
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);

    //Add ambient and diffuse lights
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientColor);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, sunColor);
    glLightfv(GL_LIGHT0, GL_SPECULAR, sunColor);
    glLightfv(GL_LIGHT0, GL_POSITION, sunPosition);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, moonColor);
    glLightfv(GL_LIGHT1, GL_SPECULAR, moonColor);
    glLightfv(GL_LIGHT1, GL_POSITION, moonPosition);

    //get current time
    current_time = (double)GetTickCount64() / 1000.0 - initial_time;

    //check if we should update in this draw call to maintain update fps
    if (current_time - previous_time >= 1.0 / update_fps) {
        elapsed_time = current_time - previous_time;
        contact_restitution = 0;

        //update derived mesh attributes in the scene
        scene.updateMeshTransformations();

        //follow collision and contact response scheme described in:
        //nonconvex rigid bodies with stacking by Guendelman et al.
        //1: save current state information of each of the bodies
        //2: temporarily move all bodies forward in time by elapsed_time
        //3: handle collisions in this forward time step, updating velocities
        //4: restore position and orientation back to before time step
        //5: new velocites mean new collisions, thus repeat steps 2-4 for a 
        //   few iterations in order to handle this
        //6: apply external forces and torque to objects, updating velocities
        //7: handle contact using steps 2-5, but with negative/zero restitution
        //7: alternatively to step 7, use a contact graph with shock propagation
        //8: update positions based on new velocities

        //save current state of scene
        scene.saveBodyStates();

        //while there is still a collision, loop collision step up to an
        //arbitrary limit here we use five as suggested in the paper above
        for (int i = 0; i < 5 && collision; i++) {
            //temporarily update the scene to step forward in time
            scene.update(elapsed_time);

            //handle collisions in the forward step and update velocities
            //note: handleCollision returns true iff there is a collision
            collision = scene.handleCollisions();

            //restore scene state to before forward step
            scene.restoreState();
        }

        //update velocities within the scene with external forces
        scene.updateVelocities(elapsed_time);

        //handle contacts in a similiar manner to collisions
        for (int i = 0; i < 10 && contact; i++) {

            //temporarily update the scene to step forward in time
            scene.update(elapsed_time);

            //handle collisions in the forward step and update velocities
            //note: handleCollision returns true iff there is a collision
            contact = scene.handleContacts(contact_restitution);

            //restore scene state to before forward step
            scene.restoreState();
        }

        //update scene with new velocities
        scene.update(elapsed_time);

        //set collision and contact to be true for next physics update
        collision = true;
        contact = true;

        //indicate current time as time of the previous update
        previous_time = current_time;
    }

    //draw the scene
    scene.draw();

    //Draw and refresh AntTweakBar
    TwDraw();
    TwRefreshBar(bar);

    //Swap Buffers for double buffering
    glutSwapBuffers();

    //Indicate need for new display() call
    glutPostRedisplay();

}

//callback function called by GLUT when window size changes
void reshape(int width_, int height_)
{
    width = width_;
    height = height_;
    // Set OpenGL viewport and camera
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (double)width / height, 1, 100);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eye_x, eye_y, eye_z, center_x, center_y, center_z, 0, 1, 0);

    //send the new window size to AntTweakBar
    TwWindowSize(width, height);
}

//callback function that rotates object 
void TW_CALL rotate(void *clientData) {
    //test scene switching
    scene.kill();
    switch (scene_id) {
    case 0:
        scene = Boxscene();
        break;
    case 1:
        scene = BallScene();
        break;
    case 2:
        scene = MultiScene();
        break;
    case 3:
        scene = MultiSceneLight();
        break;
    case 4:
        scene = SingleBoxscene();
        break;
    case 5:
        scene = PillarScene();
        break;
    case 6:
        scene = CollideScene();
        break;
    case 7:
        scene = StackingScene();
        break;
    default:
        scene = Boxscene();
        break;
    }

    //reset time
    initial_time = (double)GetTickCount64() / 1000.0;
    current_time = (double)GetTickCount64() / 1000.0 - initial_time;
    previous_time = (double)GetTickCount64() / 1000.0 - initial_time;
}

//callback function that inverts offset
void TW_CALL invert(void *clientData) {
    impulse[0] *= -1;
    impulse[1] *= -1;
    impulse[2] *= -1;
}

//callback function that zero's offset
void TW_CALL zero(void *clientData) {
    scene.applyImpulse(location, impulse, obj_id);
}

//callback function that clears impulse
void TW_CALL clear(void *clientData) {
    location[0] = 0;
    location[1] = 0;
    location[2] = 0;
    impulse[0] = 0;
    impulse[1] = 0;
    impulse[2] = 0;
}
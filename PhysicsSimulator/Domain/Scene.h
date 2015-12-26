// Author: Daren Cheng
// Class:  CS 4392
// Date:   12/7/2015

// Desc: 
// Header file that contains the Scene class
// A Scene is a collection of Meshes

//include guard
#ifndef SCENE_H_INCLUDED
#define SCENE_H_INCLUDED

//include dependancies
#include <GL/freeglut.h>
#include <math.h>
#include <time.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <SWIFT.h>

//include local dependancies
#include <Domain/Mesh.h>
#include <Domain/Rigidbody.h>

//forward declared class references
class Mesh;
class Rigidbody;

using namespace std;

struct RigidBodyState {
    void saveCurrentState(Rigidbody body);
    double orientation[4];
    double position[3];
    double p_velocity[3];
    double l_velocity[3];
};

class Scene
{
private:
    void computeImpulseMagnitude(int id, double restitution,
        double* location, double* impulse);
    void computeImpulseMagnitude(int id1, int id2, double restitution,
        double* location1, double* location2, double* normal);
    void handleCollision(int id1, int id2, 
        double* location1, double* location2, double* normal);
    void handleContact(int id1, int id2, double restitution,
        double* location1, double* location2, double* normal);
public:
    SWIFT_Scene* swift_scene;  //SWIFT scene for collision detection
    vector<Rigidbody> bodies;  //collection of bodies in the scene
    RigidBodyState* states;    //temporarily saved body states
    double gravity;            //gravity for all objects in the scene

    Scene();
    void add_body(const char* object_filename,
        double *orientation, double *offset, bool fixed, 
        double density, double *p_velocity, double *l_velocity,
        double restitution);
    void add_body(const char* object_filename, float* color,
        double *orientation, double *offset, bool fixed, 
        double density, double *p_velocity, double *l_velocity,
        double restitution);
    void copy_body(int body_id,
        double *orientation, double *offset, bool fixed, 
        double *p_velocity, double *l_velocity, double restitution);

    //scene time integration methods
    void update(double elapsed_time);
    void updateMeshTransformations();
    void applyImpulse(double *location, double *impulse, int id);

    //scene collision and contact handling methods
    void updateVelocities(double elapsed_time);
    void saveBodyStates();
    void restoreState();
    bool handleCollisions();
    bool handleContacts(double restitution);

    //scene rendering and setup methods
    void draw();
    void activate();
    void kill();
    void print();
};

#endif
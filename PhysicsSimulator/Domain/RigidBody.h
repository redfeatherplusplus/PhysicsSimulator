// Author: Daren Cheng
// Class:  CS 4392
// Date:   12/7/2015

// Desc: 
// Header file that contains the Mesh class
// A mesh is an arbitrary polygonal or triangle mesh.

//include guard
#ifndef RIGIDBODY_H_INCLUDED
#define RIGIDBODY_H_INCLUDED

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

//include local dependancies
#include <Domain/Mesh.h>

using namespace std;

class Rigidbody : public Mesh
{
protected:
    bool fixed;            //indicates if body is fixed
    double density;        //density of object
    double restitution;    //coefficient of restitution
    double mass;           //mass of object
    double tensor[9];      //inertia tensor of object
    double inv_tensor[9];  //inverse tensor
public:
    double p_velocity[3];  //linear momentum
    double l_velocity[3];  //angular momentum
    double forces[3];      //sum of all forces being applied to the body
 
    Rigidbody();
    void copy(Rigidbody &rigidbody);
    void computePhysicalProperties(ifstream &meshFile);

    //time integration methods;
    void update(double elapsed_time);
    void applyForces(double elapsed_time);
    void applyImpulse(double *location, double *impulse);

    //getter and setter methods
    void setFixed(bool fixed);
    void setDensity(double density);
    void setRestituion(double restitution);
    bool getFixed();
    double getRestitution();
    double getMass();
    double* getTensor();
    double* getInvTensor();

    //impulse and force setter methods for convience
    void setLinearVelocity(double *p_velocity);
    void setAngularVelocity(double *l_velocity);
    void setForces(double *force);

};

#endif
// Author: Daren Cheng
// Class:  CS 4392
// Date:   12/7/2015

// Desc: 
// Header file that contains the Mesh class
// A mesh is an arbitrary polygonal or triangle mesh.

//include guard
#ifndef MESH_H_INCLUDED
#define MESH_H_INCLUDED

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

//forward declared class references
class Scene;

using namespace std;

class Mesh
{
protected:
    static bool srand_initialized;  //used to only initialize srand once

    int numVertices;                  //number of verticies in the mesh
    int numFaces;                     //number of faces in  the mesh
    vector<vector<double>> vertices;  //verticies of the object 
    vector<vector<double>> normals;   //normals computed via distance from centroid
    vector<vector<int>> faces;        //faces based off of vertex references
    vector<double> centroid;          //center of 3d mesh, used for normal generation
    double orientation[4];            //orientation stored as a quaternion
    double position[3];               //xyz position of the mesh

    void getVertices(string token, ifstream &meshFile);
    void getTriangles(string token, ifstream &meshFile);
    void computeCentroid();
    void computeNormals();
    void computeLocalMatrix();
    void drawTriangle(vector<int> triangle);
public:
    int id;                  //mesh ID
    double localMatrix[16];  //derived local matrix
    double rotation[9];      //derived rotation
    double inv_rotation[9];  //derived transposed rotation
    double translation[3];   //derived translation
    float materialColor[4];  //rgb color of mesh

    //mesh manipulation methods
    Mesh();
    void copy(Mesh &mesh);
    void getMesh(ifstream &meshFile);
    void rotate(double *quaternion);
    void translate(double *vector);
    void updateTransformations(Scene *scene);

    //rendering methods
    void setColor(float* materialColor);
    void draw();
    void print();

    //getter and setter methods
    double* getOrientation();
    double* getPosition();
    void setOrientation(double *orientation);
    void setPosition(double *position);
};

#endif
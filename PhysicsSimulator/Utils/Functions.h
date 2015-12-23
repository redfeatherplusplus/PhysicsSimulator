// Author: Daren Cheng
// Class:  CS 4392
// Date:   9/20/2015

// Desc: 
// Header file that contrains method's used in Cube.cpp

//include guard
#ifndef FUNCTIONS_H_INCLUDED
#define FUNCTIONS_H_INCLUDED

//include dependancies
#include <AntTweakBar.h>
#include <GL/freeglut.h>
#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

//functions that help with basic arithmetic
double floorMod(double dividend, double divisor);

//functions that manipulate matrices
void multiplyVector3(double *matrix, double *vector);
void multiplyMatrix3(double *m1, double *m2);
void productMatrix(double *vector, double *matrix);
void crossProduct(double *v1, double *v2, double *result);
double dotProduct(double *v1, double *v2);
void normallizeQuaternion(double *q);
void matrixToQuaternion(double *matrix, double *q);
void multiplyQuaternion(double *q1, double *q2);
void multiplyMatrix(double *firstMatrix, double *homogeneousMatrix);
void multiplyVector(double *vector, double *homogeneousMatrix);
void copyVector(double *tmp, double *vector);
void initializeMatrix(double *matrix);
void printMatrix(double *matrix);
void inverse(double *matrix);
void transpose(double *tmp, double *matrix);
void invert(double *tmp, double *vector);

//functions that manipulate matrices stored in a vector
void multiplyMatrix(vector<float> &firstMatrix, vector<float> &homogeneousMatrix);
void multiplyVector(vector<double> &vec, vector<double> &homogeneousMatrix);
void multiplyVector(vector<float> &vec, vector<float> &homogeneousMatrix);
void multiplyScalar(vector<float> &vec, float &scalar);
void initializeMatrix(vector<float> &matrix);
void printMatrix(vector<float> &matrix);
void inverse(vector<float> &matrix);

//functions that help set up the rendering environment
void drawWCSAxis();
void drawOCSAxis();
void drawCube(int subdivisions);
#endif
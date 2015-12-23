// Author: Daren Cheng
// Class:  CS 4392
// Date:   12/7/2015

// Desc: 
// Header file that contrains methods use to calculate
// mass and inertia tensor from density

//include guard
#ifndef MASSPROPERTIES_H_INCLUDED
#define MASSPROPERTIES_H_INCLUDED

//include dependancies
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

using namespace std;

//fills massProperties with tensor followed by mass
void computeMassProperties(ifstream &meshFile,
    double density, double *massProperties);

#endif
// Author: Daren Cheng
// Class:  CS 4392
// Date:   10/18/2015

// Desc: 
// Implements methods given in "Functions.h"

//include interface that this file implements
#include <Utils/Functions.h>

/////////////////////////////////////////////////
//  functions that help with basic arithmetic  //
/////////////////////////////////////////////////

//modulo based on floored division
double floorMod(double dividend, double divisor) {
    return(dividend - divisor * floor(dividend / divisor));
}

//////////////////////////////////////////
//  functions that manipulate matrices  //
//////////////////////////////////////////

//multiply vector by matrix and store result in the input vector
void multiplyVector3(double *matrix, double *vector) {
    double tmp[3] = { 0 };

    //calculate result of multiplication
    tmp[0] = matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2];
    tmp[1] = matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2];
    tmp[2] = matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2];

    //store result in input vector
    vector[0] = tmp[0];
    vector[1] = tmp[1];
    vector[2] = tmp[2];
}

//multiply two matrices and store the result in the first matrix
void multiplyMatrix3(double *m1, double *m2) {
    double product[9] = { 0 };

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            //multiply column by row for the ij'th position
            for (int ij = 0; ij < 3; ij++) {
                product[3 * i + j] += m1[3 * i + ij] * m2[3 * ij + j];
            }
        }
    }

    //copy product matrix over
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m1[3 * i + j] = product[3 * i + j];
        }
    }
}

//convert vector into product matrix form
void productMatrix(double *vector, double *matrix) {
    //set diagonal to be zero
    matrix[3 * 0 + 0] = 0.0;
    matrix[3 * 1 + 1] = 0.0;
    matrix[3 * 2 + 2] = 0.0;

    //set upper right triangle
    matrix[3 * 0 + 1] = -vector[2];
    matrix[3 * 0 + 2] = vector[1];
    matrix[3 * 1 + 2] = -vector[0];

    //set lower left triangle
    matrix[3 * 1 + 0] = vector[2];
    matrix[3 * 2 + 0] = -vector[1];
    matrix[3 * 2 + 1] = vector[0];
}

//compute cross product of a two vectors
void crossProduct(double *v1, double *v2, double *result) {
    double tmp[3] = { 0 };

    //calculate result of cross product
    tmp[0] = v1[1] * v2[2] - v1[2] * v2[1];
    tmp[1] = v1[2] * v2[0] - v1[0] * v2[2];
    tmp[2] = v1[0] * v2[1] - v1[1] * v2[0];

    //store result into result vector, note we do this last incase *result = *v1 or *v2 
    result[0] = tmp[0];
    result[1] = tmp[1];
    result[2] = tmp[2];
}

//compute dot product of a two vectors
double dotProduct(double *v1, double *v2) {
    return(v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]);
}

//normalize quaternion
void normallizeQuaternion(double *q) {
    double magnitude = 0;

    //normalize quaternion to avoid numerical drift
    magnitude = pow(
        q[0] * q[0] +
        q[1] * q[1] +
        q[2] * q[2] +
        q[3] * q[3], 0.5);

    //if magnitude is zero, don't divide by magnitude 
    //since as quaternion is zero vector
    if (magnitude != 0) {
        q[0] /= magnitude;
        q[1] /= magnitude;
        q[2] /= magnitude;
        q[3] /= magnitude;
    }
}

//convert matrix into quaternion
//adapted from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
void matrixToQuaternion(double *matrix, double *q) {
    double trace = matrix[0] + matrix[4] + matrix[8];
    if (trace > 0) {
        double s = 0.5 / sqrt(trace + 1.0);
        q[0] = 0.25 / s;
        q[1] = (matrix[7] - matrix[5]) * s;
        q[2] = (matrix[2] - matrix[6]) * s;
        q[3] = (matrix[3] - matrix[1]) * s;
    }
    else {
        if (matrix[0] > matrix[4] && matrix[0] > matrix[8]) {
            double s = 2.0 * sqrt(1.0 + matrix[0] - matrix[4] - matrix[8]);
            q[0] = (matrix[7] - matrix[5]) / s;
            q[1] = 0.25 * s;
            q[2] = (matrix[1] + matrix[3]) / s;
            q[3] = (matrix[2] + matrix[6]) / s;
        }
        else if (matrix[4] > matrix[8]) {
            double s = 2.0 * sqrt(1.0 + matrix[4] - matrix[0] - matrix[8]);
            q[0] = (matrix[2] - matrix[6]) / s;
            q[1] = (matrix[1] + matrix[3]) / s;
            q[2] = 0.25 * s;
            q[3] = (matrix[5] + matrix[7]) / s;
        }
        else {
            double s = 2.0 * sqrt(1.0 + matrix[8] - matrix[0] - matrix[4]);
            q[0] = (matrix[3] - matrix[1]) / s;
            q[1] = (matrix[2] + matrix[6]) / s;
            q[2] = (matrix[5] + matrix[7]) / s;
            q[3] = 0.25 * s;
        }
    }
}

//multiply two quaternions in reverse order and store the result in the first one
void multiplyQuaternion(double *r, double *q) {
    double product[4] = { 0 };

    //compute scalar part of quaternion
    product[0] = r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3];

    //compute vector part of quaternion
    product[1] = r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2];
    product[2] = r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1];
    product[3] = r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0];

    //copy quaternion product into first quaternion
    r[0] = product[0];
    r[1] = product[1];
    r[2] = product[2];
    r[3] = product[3];
}

//multiply two matrices and store the result in the first matrix
void multiplyMatrix(double *firstMatrix, double *homogeneousMatrix) {
    double product[4 * 4] = { 0 };

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            //multiply column by row for the ij'th position
            for (int ij = 0; ij < 4; ij++) {
                product[4 * i + j] += firstMatrix[4 * i + ij] * homogeneousMatrix[4 * ij + j];
            }
        }
    }

    //copy product matrix over
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            firstMatrix[4 * i + j] = product[4 * i + j];
        }
    }
}

//multiply a vector by a matrix and store the result in the vector
void multiplyVector(double *vector, double *homogeneousMatrix) {
    double product[4] = { 0 };

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            //multiply the row by col for each element in vector 
            product[i] += vector[j] * homogeneousMatrix[4 * j + i];
        }
    }

    //copy product vector over
    for (int i = 0; i < 4; i++) {
        vector[i] = product[i];
    }
}

//copy second vector into first
void copyVector(double *tmp, double *vector) {
    tmp[0] = vector[0];
    tmp[1] = vector[1];
    tmp[2] = vector[2];
    tmp[3] = vector[3];
}

//initialize a 4x4 matrix to the identity matrix
void initializeMatrix(double *matrix) {
    for (int i = 0; i < 16; i++) {
        matrix[i] = 0.0;
    }
    matrix[4 * 0 + 0] = 1.0;
    matrix[4 * 1 + 1] = 1.0;
    matrix[4 * 2 + 2] = 1.0;
    matrix[4 * 3 + 3] = 1.0;
}

//prints a matrix
void printMatrix(double *matrix) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%f, ", matrix[4 * i + j]);
        }
        printf("\n");
    }
    printf("\n");
}

//replaces a matrix with its inverse
void inverse(double *matrix) {
    //invert translations
    matrix[4 * 3 + 0] = -matrix[4 * 3 + 0];
    matrix[4 * 3 + 1] = -matrix[4 * 3 + 1];
    matrix[4 * 3 + 2] = -matrix[4 * 3 + 2];

    //temp matrix for upper triangle of rotation matrix
    double upper_triangle[3] = { 0 };

    //invert rotations, note that for a rotation matrix R^-1 = R^T
    upper_triangle[0] = matrix[4 * 0 + 1];
    upper_triangle[1] = matrix[4 * 0 + 2];
    upper_triangle[1] = matrix[4 * 1 + 2];

    matrix[4 * 0 + 1] = matrix[4 * 1 + 0];
    matrix[4 * 0 + 2] = matrix[4 * 2 + 0];
    matrix[4 * 1 + 2] = matrix[4 * 2 + 1];

    matrix[4 * 1 + 0] = upper_triangle[0];
    matrix[4 * 2 + 0] = upper_triangle[1];
    matrix[4 * 2 + 1] = upper_triangle[2];
}

////////////////////////////////////////////////////////////
//  functions that manipulate matrices stored in vectors  //
////////////////////////////////////////////////////////////

//multiply two matrices and store the result in the first matrix
void multiplyMatrix(vector<float> &firstMatrix, vector<float> &homogeneousMatrix) {
    float product[4 * 4] = { 0 };

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            //multiply column by row for the ij'th position
            for (int ij = 0; ij < 4; ij++) {
                product[4 * i + j] += firstMatrix[4 * i + ij] * homogeneousMatrix[4 * ij + j];
            }
        }
    }

    //copy product matrix over
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            firstMatrix[4 * i + j] = product[4 * i + j];
        }
    }
}

//multiply a vector by a matrix and store the result in the vector (double)
void multiplyVector(vector<double> &vec, vector<double> &homogeneousMatrix) {
    double product[4] = { 0 };

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            //multiply the row by col for each element in vector 
            product[i] += vec[j] * homogeneousMatrix[4 * j + i];
        }
    }

    //copy product vector over
    for (int i = 0; i < 4; i++) {
        vec[i] = product[i];
    }
}

//multiply a vector by a matrix and store the result in the vector (float)
void multiplyVector(vector<float> &vec, vector<float> &homogeneousMatrix) {
    float product[4] = { 0 };

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            //multiply the row by col for each element in vector 
            product[i] += vec[j] * homogeneousMatrix[4 * j + i];
        }
    }

    //copy product vector over
    for (int i = 0; i < 4; i++) {
        vec[i] = product[i];
    }
}

//multiply a vector by a scalar
void multiplyScalar(vector<float> &vec, float &scalar) {
    for (float &element : vec) {
        element *= scalar;
    }
}

//initialize a 4x4 matrix to the identity matrix
void initializeMatrix(vector<float> &matrix) {
    for (int i = 0; i < 16; i++) {
        matrix[i] = 0.0;
    }
    matrix[4 * 0 + 0] = 1.0;
    matrix[4 * 1 + 1] = 1.0;
    matrix[4 * 2 + 2] = 1.0;
    matrix[4 * 3 + 3] = 1.0;
}

//prints a matrix
void printMatrix(vector<float> &matrix) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%f, ", matrix[4 * i + j]);
        }
        printf("\n");
    }
    printf("\n");
}

//replaces a matrix with its inverse
void inverse(vector<float> &matrix) {
    //invert translations
    matrix[4 * 3 + 0] = -matrix[4 * 3 + 0];
    matrix[4 * 3 + 1] = -matrix[4 * 3 + 1];
    matrix[4 * 3 + 2] = -matrix[4 * 3 + 2];

    //temp matrix for upper triangle of rotation matrix
    float upper_triangle[3] = { 0 };

    //invert rotations, note that for a rotation matrix R^-1 = R^T
    upper_triangle[0] = matrix[4 * 0 + 1];
    upper_triangle[1] = matrix[4 * 0 + 2];
    upper_triangle[2] = matrix[4 * 1 + 2];

    matrix[4 * 0 + 1] = matrix[4 * 1 + 0];
    matrix[4 * 0 + 2] = matrix[4 * 2 + 0];
    matrix[4 * 1 + 2] = matrix[4 * 2 + 1];

    matrix[4 * 1 + 0] = upper_triangle[0];
    matrix[4 * 2 + 0] = upper_triangle[1];
    matrix[4 * 2 + 1] = upper_triangle[2];
}

//returns transpose of second matrix in first
void transpose(double *tmp, double *matrix) {
    double upper_triangle[3] = { 0 };  //upper triangle of matrix

    //copy matrix diagonal into tmp
    tmp[3 * 0 + 0] = matrix[3 * 0 + 0];
    tmp[3 * 1 + 1] = matrix[3 * 1 + 1];
    tmp[3 * 2 + 2] = matrix[3 * 2 + 2];

    //invert rotations, note that for a rotation matrix R^-1 = R^T
    upper_triangle[0] = matrix[3 * 0 + 1];
    upper_triangle[1] = matrix[3 * 0 + 2];
    upper_triangle[2] = matrix[3 * 1 + 2];

    tmp[3 * 0 + 1] = matrix[3 * 1 + 0];
    tmp[3 * 0 + 2] = matrix[3 * 2 + 0];
    tmp[3 * 1 + 2] = matrix[3 * 2 + 1];

    tmp[3 * 1 + 0] = upper_triangle[0];
    tmp[3 * 2 + 0] = upper_triangle[1];
    tmp[3 * 2 + 1] = upper_triangle[2];
}

//inverts a vector
void invert(double *tmp,  double *vector) {
    //copy inverted vector into tmp
    tmp[0] = -vector[0];
    tmp[1] = -vector[1];
    tmp[2] = -vector[2];
}

////////////////////////////////////////////////////////////
//  functions that help set up the rendering environment  //
////////////////////////////////////////////////////////////

//draw WCS axis
void drawWCSAxis() {
    //save current rendering settings
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    //set line parameters
    glDisable(GL_LIGHTING);
    glLineWidth(2.5);

    //draw x-axis
    glPushMatrix();
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(20.0f, 0.0f, 0.0f);
    glEnd();
    glPopMatrix();

    //draw y-axis
    glPushMatrix();
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 20.0f, 0.0f);
    glEnd();
    glPopMatrix();

    //draw z-axis
    glPushMatrix();
    glColor3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 20.0f);
    glEnd();
    glPopMatrix();

    //draw zx-plane
    glLineWidth(0.5);
    glPushMatrix();
    glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
    for (int i = 0; i < 20; i += 2) {
        //z vector lines
        glBegin(GL_LINES);
        glVertex3f(i*1.0f, 0.0f, -20.0f);
        glVertex3f(i*1.0f, 0.0f, 20.0f);
        glVertex3f(-i*1.0f, 0.0f, -20.0f);
        glVertex3f(-i*1.0f, 0.0f, 20.0f);
        glEnd();

        //x vector lines
        glBegin(GL_LINES);
        glVertex3f(-20.0f, 0.0f, 1.0f*i);
        glVertex3f(20.0f, 0.0f, 1.0f*i);
        glVertex3f(-20.0f, 0.0f, -1.0f*i);
        glVertex3f(20.0f, 0.0f, -1.0f*i);
        glEnd();
    }
    glPopMatrix();

    //restore previous rendering settings
    glPopAttrib();
}

//draw OCS axis
void drawOCSAxis() {
    //save current rendering settings
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    //set line parameters
    glDisable(GL_LIGHTING);

    //draw x-axis
    glPushMatrix();
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(-1.5f, 0.0f, 0.0f);
    glVertex3f(1.5f, 0.0f, 0.0f);
    glEnd();
    glPopMatrix();

    //draw y-axis
    glPushMatrix();
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, -1.5f, 0.0f);
    glVertex3f(0.0f, 1.5f, 0.0f);
    glEnd();
    glPopMatrix();

    //draw z-axis
    glPushMatrix();
    glColor3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, -1.5f);
    glVertex3f(0.0f, 0.0f, 1.5f);
    glEnd();
    glPopMatrix();

    //re-enable lighting
    glEnable(GL_LIGHTING);

    //restore previous rendering settings
    glPopAttrib();
}

//draw cube with subdivisions
void drawCube(int subdivisions) {
    double sub_length = 1.0 / (double)subdivisions;

    //define triangles according to the right-hand rule
    glPushMatrix();
    for (int i = 0; i < subdivisions; i++) {
        for (int j = 0; j < subdivisions; j++) {
            //front and back x-face
            glBegin(GL_TRIANGLES);
            glNormal3f(1.0, 0.0, 0.0);
            glVertex3f(0.5, (i + 0)*sub_length - 0.5, (j + 0)*sub_length - 0.5);
            glVertex3f(0.5, (i + 1)*sub_length - 0.5, (j + 0)*sub_length - 0.5);
            glVertex3f(0.5, (i + 1)*sub_length - 0.5, (j + 1)*sub_length - 0.5);
            glEnd();
            glBegin(GL_TRIANGLES);
            glNormal3f(1.0, 0.0, 0.0);
            glVertex3f(0.5, (i + 1)*sub_length - 0.5, (j + 1)*sub_length - 0.5);
            glVertex3f(0.5, (i + 0)*sub_length - 0.5, (j + 1)*sub_length - 0.5);
            glVertex3f(0.5, (i + 0)*sub_length - 0.5, (j + 0)*sub_length - 0.5);
            glEnd();
            glBegin(GL_TRIANGLES);
            glNormal3f(-1.0, 0.0, 0.0);
            glVertex3f(-0.5, (i + 1)*sub_length - 0.5, (j + 1)*sub_length - 0.5);
            glVertex3f(-0.5, (i + 1)*sub_length - 0.5, (j + 0)*sub_length - 0.5);
            glVertex3f(-0.5, (i + 0)*sub_length - 0.5, (j + 0)*sub_length - 0.5);
            glEnd();
            glBegin(GL_TRIANGLES);
            glNormal3f(-1.0, 0.0, 0.0);
            glVertex3f(-0.5, (i + 0)*sub_length - 0.5, (j + 0)*sub_length - 0.5);
            glVertex3f(-0.5, (i + 0)*sub_length - 0.5, (j + 1)*sub_length - 0.5);
            glVertex3f(-0.5, (i + 1)*sub_length - 0.5, (j + 1)*sub_length - 0.5);
            glEnd(); 

            //front and back y-face
            glBegin(GL_TRIANGLES);
            glNormal3f(0.0, 1.0, 0.0);
            glVertex3f((i + 1)*sub_length - 0.5, 0.5, (j + 1)*sub_length - 0.5);
            glVertex3f((i + 1)*sub_length - 0.5, 0.5, (j + 0)*sub_length - 0.5);
            glVertex3f((i + 0)*sub_length - 0.5, 0.5, (j + 0)*sub_length - 0.5);
            glEnd();
            glBegin(GL_TRIANGLES);
            glNormal3f(0.0, 1.0, 0.0);
            glVertex3f((i + 0)*sub_length - 0.5, 0.5, (j + 0)*sub_length - 0.5);
            glVertex3f((i + 0)*sub_length - 0.5, 0.5, (j + 1)*sub_length - 0.5);
            glVertex3f((i + 1)*sub_length - 0.5, 0.5, (j + 1)*sub_length - 0.5);
            glEnd();
            glBegin(GL_TRIANGLES);
            glNormal3f(0.0, -1.0, 0.0);
            glVertex3f((i + 0)*sub_length - 0.5, -0.5, (j + 0)*sub_length - 0.5);
            glVertex3f((i + 1)*sub_length - 0.5, -0.5, (j + 0)*sub_length - 0.5);
            glVertex3f((i + 1)*sub_length - 0.5, -0.5, (j + 1)*sub_length - 0.5);
            glEnd();
            glBegin(GL_TRIANGLES);
            glNormal3f(0.0, -1.0, 0.0);
            glVertex3f((i + 1)*sub_length - 0.5, -0.5, (j + 1)*sub_length - 0.5);
            glVertex3f((i + 0)*sub_length - 0.5, -0.5, (j + 1)*sub_length - 0.5);
            glVertex3f((i + 0)*sub_length - 0.5, -0.5, (j + 0)*sub_length - 0.5);
            glEnd();

            //front and back z-face
            glBegin(GL_TRIANGLES);
            glNormal3f(0.0, 0.0, 1.0);
            glVertex3f((i + 0)*sub_length - 0.5, (j + 0)*sub_length - 0.5, 0.5);
            glVertex3f((i + 1)*sub_length - 0.5, (j + 0)*sub_length - 0.5, 0.5);
            glVertex3f((i + 1)*sub_length - 0.5, (j + 1)*sub_length - 0.5, 0.5);
            glEnd();
            glBegin(GL_TRIANGLES);
            glNormal3f(0.0, 0.0, 1.0);
            glVertex3f((i + 1)*sub_length - 0.5, (j + 1)*sub_length - 0.5, 0.5);
            glVertex3f((i + 0)*sub_length - 0.5, (j + 1)*sub_length - 0.5, 0.5);
            glVertex3f((i + 0)*sub_length - 0.5, (j + 0)*sub_length - 0.5, 0.5);
            glEnd();
            glBegin(GL_TRIANGLES);
            glNormal3f(0.0, 0.0, -1.0);
            glVertex3f((i + 1)*sub_length - 0.5, (j + 1)*sub_length - 0.5, -0.5);
            glVertex3f((i + 1)*sub_length - 0.5, (j + 0)*sub_length - 0.5, -0.5);
            glVertex3f((i + 0)*sub_length - 0.5, (j + 0)*sub_length - 0.5, -0.5);
            glEnd();
            glBegin(GL_TRIANGLES);
            glNormal3f(0.0, 0.0, -1.0);
            glVertex3f((i + 0)*sub_length - 0.5, (j + 0)*sub_length - 0.5, -0.5);
            glVertex3f((i + 0)*sub_length - 0.5, (j + 1)*sub_length - 0.5, -0.5);
            glVertex3f((i + 1)*sub_length - 0.5, (j + 1)*sub_length - 0.5, -0.5);
            glEnd();
        }
    }
    glPopMatrix();
}

// Author: Daren Cheng
// Class:  CS 4392
// Date:   10/18/2015

// Desc: 
// Implements methods given in "Mesh.h"

//include interface that this file implements and utility libraries
#include <Domain/Mesh.h>
#include <Utils/Functions.h>

//global variables
int v1;       //first vertex of triangle
int v2;       //second vertex of triangle
int v3;       //third vertex of triangle
int vertexId; //id of a vertex of a polygon

//static variable declarations
bool Mesh::srand_initialized = false;

///////////////////////
//  private methods  //
///////////////////////

//get every vertex in the mesh
void Mesh::getVertices(string token, ifstream &meshFile) {
    for (int i = 0; i < numVertices; i++) {
        //get a single vertex by getting its coordinate values
        vector<double> vertex;
        meshFile >> token;
        vertex.push_back(stod(token));
        meshFile >> token;
        vertex.push_back(stod(token));
        meshFile >> token;
        vertex.push_back(stod(token));

        //add current vertex to the list of vertices
        vertices.push_back(vertex);
    }
}

//get every triangle in the mesh
void Mesh::getTriangles(string token, ifstream &meshFile) {
    for (int i = 0; i < numFaces; i++) {
        //get a single triangle by getting its vertex id's
        vector<int> triangle;
        meshFile >> token;
        triangle.push_back(stoi(token));
        meshFile >> token;
        triangle.push_back(stoi(token));
        meshFile >> token;
        triangle.push_back(stoi(token));

        //add current triangle to the list of faces
        faces.push_back(triangle);
    }
}

//compute the centroid of the mesh
void Mesh::computeCentroid() {
    double sum_vertices_x = 0;  //sum of verticies x-component
    double sum_vertices_y = 0;  //sum of verticies y-component
    double sum_vertices_z = 0;  //sum of verticies z-component

    //compute sum of vertices component-wise
    for (int i = 0; i < numVertices; i++) {
        sum_vertices_x += vertices[i][0];
        sum_vertices_y += vertices[i][1];
        sum_vertices_z += vertices[i][2];
    }

    //average sum of vertices to compute centroid
    centroid.push_back(sum_vertices_x / numVertices);
    centroid.push_back(sum_vertices_y / numVertices);
    centroid.push_back(sum_vertices_z / numVertices);
}

//compute the normals of the mesh
void Mesh::computeNormals() {
    double magnitude = 0;  //magnitude of a normal vector

    for (int i = 0; i < numVertices; i++) {
        //compute normal direction as direction away from centroid
        vector<double> normal;
        normal.push_back(vertices[i][0] - centroid[0]);
        normal.push_back(vertices[i][1] - centroid[1]);
        normal.push_back(vertices[i][2] - centroid[2]);

        //normalize this vector
        magnitude = pow(
            normal[0] * normal[0] +
            normal[1] * normal[1] +
            normal[2] * normal[2], 0.5);
        normal[0] /= magnitude;
        normal[1] /= magnitude;
        normal[2] /= magnitude;

        //add current normal to the list of normals
        normals.push_back(normal);
    }
}

//compute local matrix from orientation quaternion and position vector
void Mesh::computeLocalMatrix() {
    //compute upper 3x3 rotation matrix
    localMatrix[0] = 1 - 2 * orientation[2] * orientation[2] - 2 * orientation[3] * orientation[3];
    localMatrix[1] = 2 * orientation[1] * orientation[2] - 2 * orientation[0] * orientation[3];
    localMatrix[2] = 2 * orientation[1] * orientation[3] + 2 * orientation[0] * orientation[2];

    localMatrix[4] = 2 * orientation[1] * orientation[2] + 2 * orientation[0] * orientation[3];
    localMatrix[5] = 1 - 2 * orientation[1] * orientation[1] - 2 * orientation[3] * orientation[3];
    localMatrix[6] = 2 * orientation[2] * orientation[3] - 2 * orientation[0] * orientation[1];

    localMatrix[8] = 2 * orientation[1] * orientation[3] - 2 * orientation[0] * orientation[2];
    localMatrix[9] = 2 * orientation[2] * orientation[3] + 2 * orientation[0] * orientation[1];
    localMatrix[10] = 1 - 2 * orientation[1] * orientation[1] - 2 * orientation[2] * orientation[2];

    //set position as specified
    localMatrix[12] = position[0];
    localMatrix[13] = position[1];
    localMatrix[14] = position[2];
}

//draw a single triangle
void Mesh::drawTriangle(vector<int> triangle) {
    //get the verticies specified by this triangle
    v1 = triangle[0];
    v2 = triangle[1];
    v3 = triangle[2];

    //first vertex
    glNormal3d(normals[v1][0], normals[v1][1], normals[v1][2]);
    glVertex3d(vertices[v1][0], vertices[v1][1], vertices[v1][2]);

    //second vertex
    glNormal3d(normals[v2][0], normals[v2][1], normals[v2][2]);
    glVertex3d(vertices[v2][0], vertices[v2][1], vertices[v2][2]);

    //third vertex
    glNormal3d(normals[v3][0], normals[v3][1], normals[v3][2]);
    glVertex3d(vertices[v3][0], vertices[v3][1], vertices[v3][2]);
}

//////////////////////
//  public methods  //
//////////////////////

//Mesh default constructor
Mesh::Mesh() {
    //initialize srand once
    if (!srand_initialized) {
        srand((int)time(NULL));
        srand_initialized = true;
    }

    //initialize orientation
    orientation[0] = 1.0;
    orientation[1] = 0.0;
    orientation[2] = 0.0;
    orientation[3] = 0.0;

    //initialize position
    position[0] = 0.0;
    position[1] = 0.0;
    position[2] = 0.0;

    //choose random color for mesh
    float red = rand() % 256 / 255.f;
    float green = rand() % 256 / 255.f;
    float blue = rand() % 256 / 255.f;

    //set material colors
    materialColor[0] = red;
    materialColor[1] = green;
    materialColor[2] = blue;
    materialColor[3] = 1.f;

    //initialize local matrix
    initializeMatrix(localMatrix);

    //initialize rotation 
    for (int i = 0; i < 9; i++) {
        rotation[i] = 0;
    }
    rotation[0] = 1;
    rotation[4] = 1;
    rotation[8] = 1;

    //initialize translation
    translation[0] = 0.0;
    translation[1] = 0.0;
    translation[2] = 0.0;
}

//copy attributes of another mesh onto this one
void Mesh::copy(Mesh &mesh) {
    //copy mesh attibutes
    numVertices = mesh.numVertices;
    numFaces = mesh.numFaces;
    vertices = mesh.vertices;
    normals = mesh.normals;
    faces = mesh.faces;
    centroid = mesh.centroid;
}

//get the mesh from the meshFile by getting it's vertices and faces
void Mesh::getMesh(ifstream &meshFile) {
    string token;

    //get the type, number of vertices, and number of faces in this mesh
    meshFile >> token;
    meshFile >> token;
    numVertices = stoi(token);
    meshFile >> token;
    numFaces = stoi(token);

    //get the verticies of this mesh
    getVertices(token, meshFile);
    getTriangles(token, meshFile);

    //compute centroid and normals of the mesh
    computeCentroid();
    computeNormals();
}

//rotate in ocs
void Mesh::rotate(double *quaternion) {
    //update current orientation by multiplying by the quaternion
    multiplyQuaternion(orientation, quaternion);
    normallizeQuaternion(orientation);
}

//translate in wcs
void Mesh::translate(double *vector) {
    position[0] += vector[0];
    position[1] += vector[1];
    position[2] += vector[2];
}

//compute rotation matrix from orientation quaternion
void Mesh::updateTransformations() {
    //note that while this method is similiar to computeLocalMatrix we have 
    //chosen against extracting a method since the methods operate on different 
    //attributes, in particular, attributes with different array indexing and size

    //compute pre-transposed 3x3 rotation matrix
    inv_rotation[0] = 1 - 2 * orientation[2] * orientation[2] - 2 * orientation[3] * orientation[3];
    inv_rotation[3] = 2 * orientation[1] * orientation[2] - 2 * orientation[0] * orientation[3];
    inv_rotation[6] = 2 * orientation[1] * orientation[3] + 2 * orientation[0] * orientation[2];

    inv_rotation[1] = 2 * orientation[1] * orientation[2] + 2 * orientation[0] * orientation[3];
    inv_rotation[4] = 1 - 2 * orientation[1] * orientation[1] - 2 * orientation[3] * orientation[3];
    inv_rotation[7] = 2 * orientation[2] * orientation[3] - 2 * orientation[0] * orientation[1];

    inv_rotation[2] = 2 * orientation[1] * orientation[3] - 2 * orientation[0] * orientation[2];
    inv_rotation[5] = 2 * orientation[2] * orientation[3] + 2 * orientation[0] * orientation[1];
    inv_rotation[8] = 1 - 2 * orientation[1] * orientation[1] - 2 * orientation[2] * orientation[2];

    //transpose inv_rotation into rotation
    transpose(rotation, inv_rotation);

    //update translation values
    translation[0] = position[0];
    translation[1] = position[1];
    translation[2] = position[2];
}

/////////////////////////
//  rendering methods  //
/////////////////////////

//set color of mesh
void Mesh::setColor(float *materialColor) {
    for (int i = 0; i < 4; i++) {
        this->materialColor[i] = materialColor[i];
    }
}

//draw entire mesh
void Mesh::draw() {
    //indicate start of mesh
    glPushMatrix();

    //move mesh into correct orientation and position
    computeLocalMatrix();
    glMultMatrixd(localMatrix);

    //set color for the triangles
    glBegin(GL_TRIANGLES);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, materialColor);

    //draw every triangle
    for (vector<int> triangle : faces) {
        drawTriangle(triangle);
    }
    glEnd();
    
    //end of mesh
    glPopMatrix();
}

//print out attribute for testing purposes
void Mesh::print() {
    //print out size of vertices and normals
    cout << "vertices size: " << vertices.size() << endl;
    for (vector<double> vertex : vertices) {
        cout << vertex[0] << ", ";
        cout << vertex[1] << ", ";
        cout << vertex[2] << ", ";
        cout << endl;
    }
    cout << endl;

    //print out face vertex id's
    cout << "faces size: " << faces.size() << endl;
    for (vector<int> face : faces) {
        for (int vertexId : face) {
            cout << vertexId << ", ";
        }
        cout << endl;
    }
    cout << endl;

    //print out and normals
    cout << "normals size: " << normals.size() << endl;
    for (vector<double> normal : normals) {
        cout << normal[0] << ", ";
        cout << normal[1] << ", ";
        cout << normal[2] << ", ";
        cout << endl;
    }
    cout << endl;

    //print out color
    cout << "colors: " << endl;
    cout << materialColor[0] << ", ";
    cout << materialColor[1] << ", ";
    cout << materialColor[2] << ", ";
    cout << endl;
}

//////////////////////
//  getter methods  //
//////////////////////

double* Mesh::getOrientation() { return orientation; }
double* Mesh::getPosition() { return position; }
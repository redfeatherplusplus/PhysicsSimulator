// Author: Daren Cheng
// Class:  CS 4392
// Date:   10/18/2015

// Desc: 
// Implements methods given in "Balljoint.h"

//include interface that this file implements and utility libraries
#include <Domain/RigidBody.h>
#include <Utils/Functions.h>
#include <PhysicsEngine/MassProperties.h>

///////////////////////
//  private methods  //
///////////////////////

//////////////////////
//  public methods  //
//////////////////////

//Rigidbody default constructor
Rigidbody::Rigidbody() {
    Mesh::Mesh();
}

//copy attibutes of another rigid body onto this one
void Rigidbody::copy(Rigidbody &Rigidbody) {
    //copy mesh attibutes
    Mesh::copy(Rigidbody);

    //copy physical attributes
    density = Rigidbody.density;
    mass = Rigidbody.mass;
    for (int i = 0; i < 9; i++) {
        tensor[i] = Rigidbody.tensor[i];
        inv_tensor[i] = Rigidbody.inv_tensor[i];
    }
    for (int i = 0; i < 3; i++) {
        forces[i] = Rigidbody.forces[i];
    }
}

//compute mass properties
void Rigidbody::computePhysicalProperties(ifstream &meshFile) {
    double massProperties[10] = { 0 };

    //computeMassProperties puts physical properties into massProperties
    computeMassProperties(meshFile, density, massProperties);

    //set mass
    mass = massProperties[9];

    //set inertia tensor and inverse tensor
    for (int i = 0; i < 9; i++) {
        tensor[i] = massProperties[i];
        inv_tensor[i] = massProperties[i];
    }

    //compute inverse tensor 
    //code adapted from: http://stackoverflow.com/questions/983999/simple-3x3-matrix-inverse-code-c

    double det =
        tensor[0] * (tensor[4] * tensor[8] - tensor[7] * tensor[5]) -
        tensor[1] * (tensor[3] * tensor[8] - tensor[5] * tensor[6]) +
        tensor[2] * (tensor[3] * tensor[7] - tensor[4] * tensor[6]);

    double invdet = 1.0 / det;
    inv_tensor[0] = (tensor[4] * tensor[8] - tensor[7] * tensor[5])* invdet;
    inv_tensor[1] = (tensor[2] * tensor[7] - tensor[1] * tensor[8])* invdet;
    inv_tensor[2] = (tensor[1] * tensor[5] - tensor[2] * tensor[4])* invdet;
    inv_tensor[3] = (tensor[5] * tensor[6] - tensor[3] * tensor[8])* invdet;
    inv_tensor[4] = (tensor[0] * tensor[8] - tensor[2] * tensor[6])* invdet;
    inv_tensor[5] = (tensor[3] * tensor[2] - tensor[0] * tensor[5])* invdet;
    inv_tensor[6] = (tensor[3] * tensor[7] - tensor[6] * tensor[4])* invdet;
    inv_tensor[7] = (tensor[6] * tensor[1] - tensor[0] * tensor[7])* invdet;
    inv_tensor[8] = (tensor[0] * tensor[4] - tensor[3] * tensor[1])* invdet;

    //print for testing
    cout << "Mass Properties for ID: " << id << endl;
    cout << "Tensor: " << endl;
    for (int i = 0; i < 9; i += 3) {
        cout << tensor[i + 0] << ", ";
        cout << tensor[i + 1] << ", ";
        cout << tensor[i + 2] << ", ";
        cout << endl;
    }
    cout << "Inverse Tensor: " << endl;
    for (int i = 0; i < 9; i += 3) {
        cout << inv_tensor[i + 0] << ", ";
        cout << inv_tensor[i + 1] << ", ";
        cout << inv_tensor[i + 2] << ", ";
        cout << endl;
    }
    cout << "Mass: " << mass << endl;
}

////////////////////////////////
//  time integration methods  //
////////////////////////////////

void Rigidbody::applyForces(double elapsed_time) {
    //update linear velocity by acceleration, note that since 
    //the only force we model is gravity, we use a simplified 
    //update of force * time
    for (int i = 0; i < 3; i++) {
        if (!fixed) {
            p_velocity[i] += forces[i] * elapsed_time;
        }
    }
}

//update the given mesh
void Rigidbody::update(double elapsed_time, Scene *scene) {
    if (!fixed) {
        //update position based on linear velocity

        for (int i = 0; i < 3; i++) {
            position[i] += p_velocity[i] * elapsed_time;
        }

        //comptute angular velocity as a quaternion
        //note: need to figure out why -l_velocity produces correct results
        //possibly due to left hand rule
        double rotation_quat[4] = { 0 };
        rotation_quat[1] = 0.5 * l_velocity[0] * elapsed_time;
        rotation_quat[2] = 0.5 * l_velocity[1] * elapsed_time;
        rotation_quat[3] = 0.5 * l_velocity[2] * elapsed_time;
        multiplyQuaternion(orientation, rotation_quat);

        //add change due to angular velocity to orientation
        //orientation[0] += 0.5 * rotation_quat[0];
        //orientation[1] += 0.5 * rotation_quat[1];
        //orientation[2] += 0.5 * rotation_quat[2];
        //orientation[3] += 0.5 * rotation_quat[3];

        //normalize orientation
        normallizeQuaternion(orientation);

        //update swift scene
        updateTransformations();
        scene->swift_scene->Set_Object_Transformation(id,
            inv_rotation,
            translation);
    }
}

//apply the given impulse at the selected location in wcs
void Rigidbody::applyImpulse(double *location, double *impulse) {
    double delta_l[3] = { 0 };
    double oriented_inv_tensor[9] = { 0 };

    //update linear velocity
    p_velocity[0] += impulse[0] / mass;
    p_velocity[1] += impulse[1] / mass;
    p_velocity[2] += impulse[2] / mass;

    //compute oriented inverse tensor, note
    //that rotation has been pre-transposed
    transpose(oriented_inv_tensor, inv_rotation);
    multiplyMatrix3(oriented_inv_tensor, inv_tensor);
    multiplyMatrix3(oriented_inv_tensor, inv_rotation);

    //compute change in angular velocity
    crossProduct(location, impulse, delta_l);
    multiplyVector3(oriented_inv_tensor, delta_l);

    //update angular velocity
    l_velocity[0] += delta_l[0];
    l_velocity[1] += delta_l[1];
    l_velocity[2] += delta_l[2];
}

/////////////////////////////////
//  getter and setter methods  //
/////////////////////////////////

void Rigidbody::setFixed(bool fixed) { this->fixed = fixed; }
void Rigidbody::setDensity(double density) { this->density = density; }
void Rigidbody::setRestituion(double restitution) { this->restitution = restitution; }

bool Rigidbody::getFixed() { return fixed; }
double Rigidbody::getRestitution() { return restitution; }
double Rigidbody::getMass() { return mass; }
double* Rigidbody::getTensor() { return tensor; }
double* Rigidbody::getInvTensor() { return inv_tensor; }

//////////////////////////////////////////////////////
//  impulse and force setter methods for convience  //
//////////////////////////////////////////////////////

//set linear velocity
void Rigidbody::setLinearVelocity(double *p_velocity) {
    this->p_velocity[0] = p_velocity[0];
    this->p_velocity[1] = p_velocity[1];
    this->p_velocity[2] = p_velocity[2];
}

//set angular velocity 
void Rigidbody::setAngularVelocity(double *l_velocity) {
    this->l_velocity[0] = l_velocity[0];
    this->l_velocity[1] = l_velocity[1];
    this->l_velocity[2] = l_velocity[2];
}

//set external forces
void Rigidbody::setForces(double *force) {
    this->forces[0] = force[0];
    this->forces[1] = force[1];
    this->forces[2] = force[2];
}

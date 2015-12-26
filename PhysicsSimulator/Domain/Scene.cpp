// Author: Daren Cheng
// Class:  CS 4392
// Date:   10/18/2015

// Desc: 
// Implements methods given in "Scene.h"

//include interface that this file implements and utility libraries
#include <Domain/Scene.h>
#include <Utils/Functions.h>

const double THRESHHOLD = 0.1;
double *old_orientation;
double *old_position;
double *old_p_velocity;
double *old_l_velocity;

////////////////////////////////////
//  RigidBodyState struct methods //
////////////////////////////////////

//save state of the given rigid body
void RigidBodyState::saveCurrentState(Rigidbody body) {
    //get pointers to each of the given body's attributes
    old_orientation = body.getOrientation();
    old_position = body.getPosition();
    old_p_velocity = body.p_velocity;
    old_l_velocity = body.l_velocity;

    //save current state information
    for (int i = 0; i < 4; i++) {
        orientation[i] = old_orientation[i];
    }
    for (int i = 0; i < 3; i++) {
        position[i] = old_position[i];
        p_velocity[i] = old_p_velocity[i];
        l_velocity[i] = old_l_velocity[i];
    }
}

///////////////////////
//  private methods  //
///////////////////////

//compute magnitude of an impulse against a static body
void Scene::computeImpulseMagnitude(int id, double restitution,
    double* location, double* impulse) {

    double oriented_inv_tensor[9] = { 0 };
    double relative_velocity[3] = { 0 };
    double tmp_vec[3] = { 0 };
    double angular = 0;
    double magnitude = 0;

    //orient inverse tensor
    transpose(oriented_inv_tensor, bodies[id].rotation);
    multiplyMatrix3(oriented_inv_tensor, bodies[id].getInvTensor());
    multiplyMatrix3(oriented_inv_tensor, bodies[id].rotation);

    //note that angular contribution to magnitude denominator from a 
    //body is: normal * [inv_tensor x (location x normal)] x location 

    //compute angular contribution to denominator from body
    crossProduct(location, impulse, tmp_vec);
    multiplyVector3(oriented_inv_tensor, tmp_vec);
    crossProduct(tmp_vec, location, tmp_vec);
    angular = dotProduct(impulse, tmp_vec);
    angular = 0;

    //compute contribution to magnitude from relative velocity
    for (int i = 0; i < 3; i++) {
        relative_velocity[i] += bodies[id].p_velocity[i];
    }
    crossProduct(bodies[id].l_velocity, location, tmp_vec);
    //relative_velocity[0] += tmp_vec[0];
    //relative_velocity[1] += tmp_vec[1];
    //relative_velocity[2] += tmp_vec[2];

    //compute numerator of magnitude based on lowest coefficent of restitution
    magnitude = -(1.0 + restitution) * dotProduct(relative_velocity, impulse);

    //divide numerator of magnitude by denominator
    magnitude /= (1.0 / bodies[id].getMass()) + angular;

    //update impulse by magnitude
    impulse[0] *= magnitude;
    impulse[1] *= magnitude;
    impulse[2] *= magnitude;
}

//compute magnitude of an impulse
void Scene::computeImpulseMagnitude(int id1, int id2, double restitution,
    double* location1, double* location2, double* impulse) {

    double oriented_inv_tensor[9] = { 0 };
    double v_relative[3] = { 0 };
    double tmp_vec[3] = { 0 };
    double angular1 = 0;
    double angular2 = 0;
    double magnitude = 0;

    //note that angular contribution to magnitude denominator from a 
    //body is: normal * [inv_tensor x (location x normal)] x location 

    //compute oriented inverse tensor for first body
    bodies[id1].updateTransformations();
    transpose(oriented_inv_tensor, bodies[id1].rotation);
    multiplyMatrix3(oriented_inv_tensor, bodies[id1].getInvTensor());
    multiplyMatrix3(oriented_inv_tensor, bodies[id1].rotation);

    //compute angular contribution from first body
    crossProduct(location1, impulse, tmp_vec);
    multiplyVector3(oriented_inv_tensor, tmp_vec);
    crossProduct(tmp_vec, location1, tmp_vec);
    angular1 = dotProduct(impulse, tmp_vec);
    angular1 = 0;

    //compute oriented inverse tensor for second body
    bodies[id2].updateTransformations();
    transpose(oriented_inv_tensor, bodies[id2].rotation);
    multiplyMatrix3(oriented_inv_tensor, bodies[id2].getInvTensor());
    multiplyMatrix3(oriented_inv_tensor, bodies[id2].rotation);

    //compute angular contribution from second body
    crossProduct(location2, impulse, tmp_vec);
    multiplyVector3(oriented_inv_tensor, tmp_vec);
    crossProduct(tmp_vec, location2, tmp_vec);
    angular2 = dotProduct(impulse, tmp_vec);
    angular2 = 0;

    //compute contribution to magnitude from relative velocity
    for (int i = 0; i < 3; i++) {
        v_relative[i] += bodies[id1].p_velocity[i];
        v_relative[i] -= bodies[id2].p_velocity[i];
    }
    crossProduct(bodies[id1].l_velocity, location1, tmp_vec);
    //v_relative[0] += tmp_vec[0];
    //v_relative[1] += tmp_vec[1];
    //v_relative[2] += tmp_vec[2];
    crossProduct(bodies[id2].l_velocity, location2, tmp_vec);
    //v_relative[0] -= tmp_vec[0];
    //v_relative[1] -= tmp_vec[1];
    //v_relative[2] -= tmp_vec[2];
    
    //compute numerator of magnitude based coefficent of restitution
    magnitude = -(1.0 + restitution) * dotProduct(v_relative, impulse);

    //divide numerator of magnitude by denominator
    magnitude /= (1.0 / bodies[id1].getMass()) + (1.0 / bodies[id2].getMass()) + angular1 + angular2;

    //update impulse by magnitude
    impulse[0] *= magnitude;
    impulse[1] *= magnitude;
    impulse[2] *= magnitude;
}

//handle a single collision between two objects
void Scene::handleCollision(int id1, int id2, 
    double* location1, double* location2, double* normal) {

    //check if either of the colliding bodies are static
    //note that both may not be static, since static to
    //static collisions are not reported
    if (bodies[id1].getFixed()) {
        //apply impulse to other non-static body
        double impulse[3] = { 0 }; 

        //location of impact is given in terms of ocs, orient to wcs
        multiplyVector3(bodies[id2].rotation, location2);

        //copy normal direction into impulse
        impulse[0] = normal[0];
        impulse[1] = normal[1];
        impulse[2] = normal[2];

        //compute impulse magnitude
        computeImpulseMagnitude(id2, bodies[id2].getRestitution(),
            location2, impulse);

        //apply impulse
        bodies[id2].applyImpulse(location2, impulse);
    }
    else if (bodies[id2].getFixed()) {
        //apply impulse to other non-static body
        double impulse[3] = { 0 };  

        //location of impact is given in terms of ocs, orient to wcs
        multiplyVector3(bodies[id1].rotation, location1);

        //copy normal direction into impulse
        impulse[0] = -normal[0];
        impulse[1] = -normal[1];
        impulse[2] = -normal[2];

        //compute impulse magnitude
        computeImpulseMagnitude(id1, bodies[id1].getRestitution(),
            location1, impulse);

        //apply impulse   
        bodies[id1].applyImpulse(location1, impulse);
    }
    else {
        //apply impluses to both bodies
        double impulse1[3] = { 0 };
        double impulse2[3] = { 0 };
        double restitution = 0;

        //location of impacts are given in terms of ocs, orient to wcs
        multiplyVector3(bodies[id1].rotation, location1);
        multiplyVector3(bodies[id2].rotation, location2);

        //copy normal directions into impulses
        impulse1[0] = -normal[0];
        impulse1[1] = -normal[1];
        impulse1[2] = -normal[2];
        impulse2[0] = normal[0];
        impulse2[1] = normal[1];
        impulse2[2] = normal[2];

        //find and use minimum restitution
        if (bodies[id1].getRestitution() <= bodies[id2].getRestitution()) {
            restitution = bodies[id1].getRestitution();
        }
        else {
            restitution = bodies[id2].getRestitution();
        }

        //compute impulse magnitudes
        computeImpulseMagnitude(id1, id2, restitution, 
            location1, location2, impulse1);
        computeImpulseMagnitude(id2, id1, restitution,
            location2, location1, impulse2);

        //apply impulses
        bodies[id1].applyImpulse(location1, impulse1);
        bodies[id2].applyImpulse(location2, impulse2);
    }
}

//handle a single contact between two objects
void Scene::handleContact(int id1, int id2, double restitution, 
    double* location1, double* location2, double* normal) {

    //check if either of the colliding bodies are static
    //note that both may not be static, since static to
    //static collisions are not reported
    if (bodies[id1].getFixed()) {
        //apply impulse to other non-static body
        double impulse[3] = { 0 };

        //location of impact is given in terms of ocs, orient to wcs
        multiplyVector3(bodies[id2].rotation, location2);

        //copy normal direction into impulse
        impulse[0] = normal[0];
        impulse[1] = normal[1];
        impulse[2] = normal[2];

        //compute impulse magnitude
        computeImpulseMagnitude(id2, restitution,
            location2, impulse);

        //apply impulse
        bodies[id2].applyImpulse(location2, impulse);
    }
    else if (bodies[id2].getFixed()) {
        //apply impulse to other non-static body
        double impulse[3] = { 0 };

        //location of impact is given in terms of ocs, orient to wcs
        multiplyVector3(bodies[id1].rotation, location1);

        //copy normal direction into impulse
        impulse[0] = -normal[0];
        impulse[1] = -normal[1];
        impulse[2] = -normal[2];

        //compute impulse magnitude
        computeImpulseMagnitude(id1, restitution,
            location1, impulse);

        //apply impulse   
        bodies[id1].applyImpulse(location1, impulse);
    }
    else {
        //apply impluses to both bodies
        double impulse1[3] = { 0 };
        double impulse2[3] = { 0 };

        //location of impacts are given in terms of ocs, orient to wcs
        multiplyVector3(bodies[id1].rotation, location1);
        multiplyVector3(bodies[id2].rotation, location2);

        //copy normal directions into impulses
        impulse1[0] = -normal[0];
        impulse1[1] = -normal[1];
        impulse1[2] = -normal[2];
        impulse2[0] = normal[0];
        impulse2[1] = normal[1];
        impulse2[2] = normal[2];

        //compute impulse magnitudes
        computeImpulseMagnitude(id1, id2, restitution,
            location1, location2, impulse1);
        computeImpulseMagnitude(id2, id1, restitution,
            location2, location1, impulse2);

        //apply impulses
        bodies[id1].applyImpulse(location1, impulse1);
        bodies[id2].applyImpulse(location2, impulse2);
    }
}

//////////////////////
//  public methods  //
//////////////////////

//Scene default constructor
Scene::Scene() {
    //create a swift scene with local bounding box sorting
    swift_scene = new SWIFT_Scene(DEFAULT_BP, DEFAULT_GS);

    //set gravity to default
    gravity = -10;
}

//add a body to the list of bodies 
void Scene::add_body(const char* object_filename,
    double *orientation, double *offset, bool fixed,
    double density, double *p_velocity, double *l_velocity, 
    double restitution) {
    //open and read the meshfile
    ifstream meshFile;
    meshFile.open(object_filename);

    //parse file into a new instance of Rigidbody
    Rigidbody body;
    if (meshFile.is_open()) {
        //read mesh attributes from file
        body.getMesh(meshFile);

        //set density of mesh
        body.setDensity(density);

        //clear stream and re-use to compute physical properties
        meshFile.clear();
        meshFile.seekg(0, ios::beg);
        body.computePhysicalProperties(meshFile);
    }
    meshFile.close();

    //perform initial offset and set orientation for mesh
    body.translate(offset);
    body.rotate(orientation);
    body.setFixed(fixed);

    //add object to SWIFT scene
    if (!swift_scene->Add_Object(object_filename, body.id, fixed,
        DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, DEFAULT_SCALE,
        DEFAULT_BOX_SETTING, DEFAULT_BOX_ENLARGE_REL, DEFAULT_BOX_ENLARGE_ABS)
        ) {
        cout << "Error, adding object. Exiting..." << endl;
        exit(-1);
    }
    cout << "ID: " << body.id << endl;

    //set the initial transformations for object in SWIFT
    body.updateTransformations();
    swift_scene->Set_Object_Transformation(body.id, 
        body.inv_rotation, body.translation);

    //let gravity be sum of all forces for the object, 
    //unless it's a fixed object
    double gravity[3] = { 0 };
    if (fixed) {
        body.setForces(gravity);
    }
    else {
        gravity[1] = this->gravity;
        body.setForces(gravity);
    }

    //set angular, linear velocity, and restitution for the mesh
    body.setLinearVelocity(p_velocity);
    body.setAngularVelocity(l_velocity);
    body.setRestituion(restitution);

    //add this mesh to the list of bodies
    bodies.push_back(body);
}

//add a body to the list of bodies with the set color
void Scene::add_body(const char* object_filename, float* color,
    double *orientation, double *offset, bool fixed, 
    double density, double *p_velocity, double *l_velocity, 
    double restitution) {

    //add the body normally
    add_body(object_filename, orientation, offset, fixed, 
        density, p_velocity, l_velocity, restitution);

    //set color of the body
    bodies.back().setColor(color);
}

//copy body then add copy to the list of bodies 
void Scene::copy_body(int body_id,
    double *orientation, double *offset, bool fixed, 
    double *p_velocity, double *l_velocity, double restitution) {
    //create mesh to copy original mesh into
    Rigidbody copy_body;

    //copy body with matching id into the copy body
    copy_body.copy(bodies[body_id]);

    //perform initial offset and set orientation for copy mesh
    copy_body.translate(offset);
    copy_body.rotate(orientation);
    copy_body.setFixed(fixed);
    copy_body.setRestituion(restitution);

    //set linear and angular velocity
    copy_body.setLinearVelocity(p_velocity);
    copy_body.setAngularVelocity(l_velocity);

    //copy object to swift scene
    if (!swift_scene->Add_Object(NULL, NULL, 0, 0, copy_body.id, fixed,
        DEFAULT_ORIENTATION, DEFAULT_TRANSLATION,
        DEFAULT_SCALE, DEFAULT_BOX_SETTING,
        DEFAULT_BOX_ENLARGE_REL, DEFAULT_BOX_ENLARGE_ABS,
        DEFAULT_FACE_VALENCES, body_id, 0)
        ) {
    }
    cout << "ID: " << copy_body.id << endl;

    //set the initial transformations for object in SWIFT
    copy_body.updateTransformations();
    swift_scene->Set_Object_Transformation(copy_body.id,
        copy_body.inv_rotation, copy_body.translation);
    
    //add the copied mesh to the list of bodies
    bodies.push_back(copy_body);
}

//////////////////////////////////////
//  scene time integration methods  //
//////////////////////////////////////

//update all objects in scene by update interval
void Scene::update(double elapsed_time) {
    for (Rigidbody &body : bodies) {
        body.update(elapsed_time, this);
    }
}

//update all derived mesh transformations in the scene
void Scene::updateMeshTransformations() {
    for (Rigidbody &body : bodies) {
        body.updateTransformations();
    }
}

//apply given impulse to the mesh with matching id
void Scene::applyImpulse(double *location, double *impulse, int id) {
    //update the mesh's derivied attributes before applying the impulse
    bodies[id].updateTransformations();

    //apply the given impulse
    bodies[id].applyImpulse(location, impulse);
}

////////////////////////////////////////////////////
//  scene collision and contact handling methods  //
////////////////////////////////////////////////////

//update velocity based on forces present in the scene
void Scene::updateVelocities(double elapsed_time) {
    for (Rigidbody &body : bodies) {
        body.applyForces(elapsed_time);
    }
}

//save current scene state by saving body states
void Scene::saveBodyStates() {
    for (int i = 0; i < bodies.size(); i++) {
        states[i].saveCurrentState(bodies[i]);
    }
}

//restore orientation and position from before timestep
void Scene::restoreState() {
    for (int i = 0; i < bodies.size(); i++) {
        bodies[i].setOrientation(states[i].orientation);
        bodies[i].setPosition(states[i].position);
    }
}

//handle collisions in the scene, returns true if collision found
bool Scene::handleCollisions() {
    int num_pairs;
    int* ids;
    double *nearest_pts;
    double *normals;

    //check for collisions
    if (swift_scene->Query_Contact_Determination(false, SWIFT_INFINITY,
        num_pairs, &ids, NO_DISTANCES, &nearest_pts, &normals)) {
        //collision found, handle each collision pair
        for (int i = 0; i < num_pairs; i++) {
            //handle a single collision pair, note we use pointer 
            //arithmetic to pass the correct normals/nearest points
            handleCollision(ids[2 * i], ids[2 * i + 1],
                nearest_pts + (6 * i), nearest_pts + (6 * i + 3),
                normals + (3 * i));
        }

        //return true to indicate that a collision was found
        return (true);
    }
    else {
        //no collision found, return false to indicate no collisions
        return(false);
    }
}

//handle contacts in the scene, returns true if contact found
bool Scene::handleContacts(double restitution) {
    int num_pairs;
    int* ids;
    double *nearest_pts;
    double *normals;

    //check for contacts
    if (swift_scene->Query_Contact_Determination(false, SWIFT_INFINITY,
        num_pairs, &ids, NO_DISTANCES, &nearest_pts, &normals)) {
        //contact found, handle each contact pair
        for (int i = 0; i < num_pairs; i++) {
            //handle a single contact pair, note we use pointer 
            //arithmetic to pass the correct normals/nearest points
            handleContact(ids[2 * i], ids[2 * i + 1], restitution, 
                nearest_pts + (6 * i), nearest_pts + (6 * i + 3),
                normals + (3 * i));
        }

        //return true to indicate that a contact was found
        return (true);
    }
    else {
        //no contact found, return false to indicate no contacts
        return(false);
    }
}

/////////////////////////////////////////
//  scene rendering and setup methods  //
/////////////////////////////////////////

//draw the current scene
void Scene::draw() {
    for (Rigidbody &body : bodies) {
        body.draw();
    }
}

//activate all pairs for collision checking in the scene and create array of body states
void Scene::activate() {
    swift_scene->Activate();

    //create an array of RigidBodyStates to hold state information for the scene
    states = new RigidBodyState[bodies.size()];
}

//delete SWIFT_Scene for the scene and delete array of body states
void Scene::kill() {
    delete swift_scene;
    delete states;
}

//print scene info
void Scene::print() {
    for (Mesh mesh : bodies) {
        cout << "id: " << mesh.id << endl;
    }
}
# Physics Simulator

![Demo](https://i.gyazo.com/436580f7238fbbf30f13cc567b47d55f.gif)

This program performs impulse-based rigid body simulation on any convex rigid body. It is based off the [Rigid Body Simulation](http://www.pixar.com/companyinfo/research/pbm2001/pdf/notesg.pdf) paper by David Baraff at Pixar and the [Nonconvex Rigid Bodies with Stacking](http://graphics.stanford.edu/papers/rigid_bodies-sig03/) paper by Eran Guendelman at Stanford.

## Domain Model

![Domain Model](https://i.imgur.com/1NJNBj9.png)

There are three key independent entities. A rendering engine, collision detector, and collision handler. 
The physics engine provides a collision detector and collision handler which manipulate objects in a scene.
The physics simulator itself is responsible for two things. Telling its physics engine to manipulate the scene.
And using some sort of rendering engine to render the scene. Lastly, a scene maintains multiple particle systems. 
In this case, rigid bodies.

I use an OTS solution for collision detection called SWIFT developed by The University of North Carolina.

## Logical View

![Logical View](https://i.imgur.com/Fmlm9oM.png)

In implementation, a Mesh superclass has been extracted out of the Rigidbody class. 
This decouples information exclusive to physical simulation from information needed for mesh rendering.
A RigidBodyState class was deemed necessary as state information is temporarily lost when processing collisions.
Lastly, unique scenes are subclasses of a generic Scene. This allows for Scene's to be swapped out in real-time.
As the engine is only aware that it is operating on a scene, it does not care which scene is being manipulated.

## TODO

The program cannot progress further without replacing the SWIFT Collision Detector. 
There is a bug where SWIFT returns non-existant collisions when more than two objects are colliding at the same time.
SWIFT is also documented to return a random point as the point of contact whenever there is more than one point of contact.
This leads to inaccurate face-to-face collisions. As a random point is chosen as the point of collision rather than the average point of contact.

As a replacement I have considered using binary space trees, axis-aligned bounding boxes, and signed distance fields as primary, second, and third levels of contact detection respectively. Alternatively attempting to utilize a different OTS solution such as Bullet may yield promising results.

Also, some decoupling must be done that was not done earlier due to time constraints. Currently the collision response logic is contained within the Scene. This logic needs to be extracted into its own class and moved into its own collision handler class.

Lastly, work should be done to replace my own library of matrix functions with a well tested matrix library such as GLM.

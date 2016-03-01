# Physics Simulator
This program performs impulse-based rigid body simulation. It is based off the [Rigid Body Simulation](http://www.pixar.com/companyinfo/research/pbm2001/pdf/notesg.pdf) paper by David Baraff at Pixar and the [Nonconvex Rigid Bodies with Stacking](http://graphics.stanford.edu/papers/rigid_bodies-sig03/) paper by Eran Guendelman at Stanford.

## Domain Model

![Domain Model](http://i390.photobucket.com/albums/oo349/thebirdislost54/physDomain03_zpshy81asa0.png~original)

There are three key independent entities. A rendering engine, collision detector, and collision handler. 
The physics engine provides a collision detector and collision handler which manipulate objects in a scene.
The physics simulator itself is responsible for using a rendering engine to render the scene.
Lastly, a scene maintains multiple particle systems. In this case, rigid bodies.

## Logical View

![Logical View](http://i390.photobucket.com/albums/oo349/thebirdislost54/physLogical02_zpsrxkgc9tv.png~original)

In implementation, a Mesh superclass has been extracted out of the Rigidbody class. 
This decouples information exclusive to physical simulation from information needed for mesh rendering.
A RigidBodyState class was deemed necessary as state information is temporarily lost when processing collisions.
Lastly, unique scenes are subclasses of a generic Scene. This allows for Scene's to be swapped out in real-time.
As the engine is only aware that it is operating on a scene, it does not care which scene is being manipulated.

## TODO

The program cannot progress further without replacing the SWIFT Collision Detector. 
I believe I have found a bug where SWIFT returns non-existant collisions when more than two objects are colliding at the same time.
SWIFT is also documented to return a random point as the point of contact whenever there is more than one point of contact.
This leads to inaccurate face-to-face collisions. As a random point is chosen as the point of collision rather than the average point of contact.

As a replacement utilizing binary space trees, axis-aligned bounding boxes, and signed distance fields as primary, second, and third levels of contact detections respectively seems promising. Alternatively attempting to utilize a different OTS solution such as Bullet may yield promising results.


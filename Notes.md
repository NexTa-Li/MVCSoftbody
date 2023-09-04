# MVC

### **_model_**

- Handles data logic
- interacts with database (in this case there isn't one)

### **_view_**

-

### **_controller_**

- Handles request flow
- never handles data logic
- Control only "controls" the Model whose state is displayed by the View

# Potential Changes:

- explore moving handleSoftbodyCollisions to the model, instead of having every softbody control that themselves. This way we might be able to find some clever solution that lets us skip collisions that may have already been handled or even explore spatial partitioning.

- Change in `Spring.java`, that would have each spring store a line between its two points instead of the indexes of only storing the index of its two points. This change might mean that we have to make our own version of Line2D where we add fields for index of each point.

- Change in `SoftBody.java`, to the `integrateHuen()` method. Fix this method, it should more accurately represent 2nd order motion integration.

- Spacial partitioning, quadtrees, or some other data structure.

- Create a CollisionHandler Utility class, the code will be a little cleaner.

### Friction changes

- change friction so that there is also an Air friction force applied every tick unless the point is already collided with a rigid surface.

### Collision Changes:

- In general the collisions should probably be handled in a different way, for example if one body is colliding with another, then the other body should be notified of the collision and then it should handle it.

- Currently a colliding point will move the edge points both a different amount depending on how far each is from the closest point. This is fine but due to the rest of the implementation, specifically how check collision iterates through each edge

### Bounding Box:

- create Rectangle class with doubles instead of ints, I suspect that the bounding box is causing some missed collisions due to rounding errors, especially when there are many bodies packed together.

# Bugs

- Two bodies at the exact same height can cause a missed collision, I suspect this is because the collision detector doesn't check for intersections through points

- Seems like during vertex collisions, they end up getting stuck together permanently in some rare instances (this may be fixed?)

- Right leaning bias (Serious bug, needs to be fixed)

- Spawning on top of each other causes a permanent stick

### Unknown bias towards positive infinity

- For the longest time there has always been a bias towards the right side of the container during sustained collisions. so if theres enough bodies to fill up the entire ground, they will force each other towards the right. I am not sure why. (easy hack is to just set some gravity towards the left)
- After doing some testing the only thing I can see this being caused by is somewhere in handleSoftBodyCollisions()
- accumulating softbody collision forces makes this less apparent with larger numbers for some reason, now they flow back and forth

#### **What is NOT causing this bug:**

- [x] applyFriction
- [x] handlePointCollision
- [ ] handleRigidCollisions ~ hard to say for sure but I doubt it

### Bounciness modifier is just a hack.

- The system is introducing a lot of extra energy somehow.
- I suspect this is due to handling the collisions inside of the integration, but if not in there then where else?
- I can move handle softbody collisions to accumulate forces and change the resolved velocities to forces, this would mean resolving softbody collisions two times per idle call.
- Update [ 2023-09-02 ]: this is driving me insane. it could be being caused by so many different things. Even a slight miscalculation in positions could cause a chain reaction of collisions permanently that could very easily be the reason why there is so much extra energy.

# Math

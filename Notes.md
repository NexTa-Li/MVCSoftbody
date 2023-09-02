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

# Math

### **_Spring Force_**

- The equation for the total force: $F_{\text{total}} = F_{\text{spring}} + F_{\text{damping}}$
- The spring force equation: $F_{\text{spring}} = -k \cdot x$
- The damping force equation: $F_{\text{damping}} = -c \cdot v$

#### Variables and Definitions

- $F_{\text{spring}}$ : Spring force
- $F_{\text{damping}}$ : Damping force
- $k$ : Spring constant
- $x$ : Displacement from the equilibrium position of the spring
- $c$ : Damping coefficient
- $v$ : Velocity of the points
- $x_1$ : Position of point 1
- $x_2$ : Position of point 2
- $v_1$ : Velocity of point 1
- $v_2$ : Velocity of point 2

#### Calculation of Displacement and Average Velocity

- Displacement: $x = x_1 - x_2$
- Average velocity: $v = \frac{v_1 + v_2}{2}$

#### Total Force Calculation

- Total force: $F_{\text{total}} = -k \cdot x - c \cdot v$

- The negative sign in the equations indicates that the forces oppose the direction of displacement and velocity.
- These equations assume a linear spring and damping behavior. If the system is more complex, the equations might differ.
- Ensure that the units for the variables and constants are consistent for accurate calculations.

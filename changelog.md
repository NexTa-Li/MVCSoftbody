# **[Version 0.0.1]** - 2023-09-02

### Changed

- `Debugging Configuration`: Changed the project from debugging config so that it runs as intended via Driver.java

# **[Version 0.0.2]** - 2023-09-02

### Removed

- `Artifacts` in `checkCollisions()` & `handleSoftbodyCollisions()`: Removing some code artifacts that were left over from old ideas/ debugging.

# **[Version 0.0.3]** - 2023-09-02

### Added

- Graphics booleans to control what parts of the softbodies are displayed.

### Changed

#### Softbody.java

- `integrateHuen()`: Temporarily removed friction from the integration, while debugging.

- `handleSoftBodyCollisions()`: Uncommented the code that changed the positions of the two points on the edge that was collided with. This was commented out for debugging purposes. This code will most likely be removed in the future.

- `handleSoftBodyCollisions()`: Made some changes to how the algorithm resolves edge point positions. These changes show a lot of promise in terms of the simulations stability and accuracy. This will still most likely be further tuned and maybe even fully reconsidered.

### Fixed

- `checkCollisions()` in `SoftBody.java`: Fixed a bug with the ray casting algorithm that caused it to miss collisions where the ray was cast through a point. When a ray was cast through a point, the algorithm would count an intersection for both edges of the point, causing the algorithm to miss the collision.

### Removed

- `accumulateSoftBodyCollisionForce()`: Removed this method. It may be added back in the future but for now an implementation that doesn't require it is being explored.

# **[Version 1.0.0]** - 2023-09-04

### Added

- `isMerged()` in `SoftBodyUtil.java`: Added a method that returns true if a softbody is inside of another softbody by checking how far apart the centre of their bounding boxes are.

- `Rectangle.java`: Added a class that represents a rectangle with double precision.

- `drawBoundingBox` bool in `SoftBodyModel.java`: Added a boolean that controls whether or not the bounding box is drawn.

### Changed

- `createSoftBody()` in `SoftBody.java`: modified the method to keep track of the bounding box, this supports the new `isMerged()` method, so it can be used from the very first tick.

- `getBoundingBox()` in `SoftBody.java`: implemented the method, returns the bounding box of the softbody.

- All `int` variables for bounding box calculations in `SoftBody.java` to `double` variables, and converted the bounding box from `java.awt.Rectangle` to `Rectangle.java`.

### Fixed

- The infamous Right leaning bias during collisions bug has finally been conqueured with the adition of higher precision variables for the rectangle class (used for bounding boxes)
  . This explains why it was more difficult to push a body from the right side to the left, because its bounding box was always smaller on the right side. so a max bound x position of 100.99 would be truncated to
  100, which means if an edge was flat past the bounding box it would be impossible to push the body to the left. It also makes it easier to push bodies from the left to right, since their bounding box would be truncated up to 0.99 units left of the actual left-most point. This is no longer the case. To make sure this definitely isn't still happening I could also add a slight margin to the bounding box, but I don't think it's necessary. Anyways this was bothering me forever so I'm very excited that this is finally fixed.

### TODO:

- `Test` that the new bounding box is working as intended.

- `Test` that the new `isMerged()` method is working as intended.

- `Test` that `closestPointOnLineSegment()` in `SoftBodyUtil.java` is working as intended.

- `Change` the display implementation so that `view` handles all the drawing instead of telling each softbody to draw itself.

- Use the new `isMerged()` method to prevent softbodies from getting trapped inside each other

# **[Version 1.0.1]** - 2023-09-04

### Changed

- `integrateHuen()` in `SoftBody.java`: Removed the 2nd call of collision resolver methods, and also changed the bounding box tracking from being in both loops to only after the points are moved in the 2nd loop. This is because the bounding box is only needed after the points are moved, and it is more efficient to only calculate it once.

# **[Version 1.1.0]** - 2023-09-05

### Changed

- `checkCollision()` in `SoftBody.java`: Cleaned up the method, made some optimizations.

- `checkCollision()` in `SoftBody.java`: Made a change to how the method detects whether a vertex collision occured (removing the extra loop with comments about it being a naive approach), method should be close to twice as fast (still needs to be tested).

- **NOTE**: for some reason the above changes made the program 10% slower but the bodies are significantly more stable. I don't understand why at all, but the difference in stability is so significant that I'm going to keep the changes.

- `checkCollision()` in `SoftBody.java`: got rid of the collided boolean, it was unnecessary.

- `checkIntersection()` in `SoftBodyUtil.java`: Changed the parameters to take a line segment instead of two points for the ray. This is because the ray is always a line segment, and it makes the method more readable. Minimal performance gain.

- `handleSoftBodyCollisions()` in `SoftBody.java`: small tweak to the handle vertex collisions portion of the method.

# **[Version 1.1.1]** - 2023-09-06

### Changed

- `various constants` in `ModelConfig.java`: tweaked the constants to test the new collision changes.

- `createSoftBodies()` in `SoftBodyModel.java`: added a temporary solution to prevent softbodies from being created inside each other. This will be replaced with a more robust solution in the future.

- `eqauls()` in `Point2D.java`: slightly modified the contract.

# **[Version 1.1.2]** - 2023-09-06

### Added

- `getXArr() & getYArr()` in `ReadOnlySoftBody.java`: Added methods that return an array of the x and y positions of the points in the softbody, so that the view can draw the softbody as a polygon.

### Changed

- `SoftBody Drawing Methods`: Now view takes the information from the model and draws the softbodies as polygons instead of asking each softbody to draw itself.

### Removed

- `PaintComponent()` in `ReadOnlySoftBody.java` & `SoftBody.java`: Removed it since the view now draws the softbodies on its own based on model data.

# **[Version 1.1.3]** - 2023-09-06

### Added

- `increase` & `decrease` in `SoftBody.java`: Added the ability to increase/ decrease spring constant, length or pressure of the selected softbody by pressing the `+` or `-` keys.

- `massChange`, `pressureChange`, `springLengthChange` & `springConstantChange` in `SoftBody.java`: toggle each option by pressing `7, 8, 9, 0` respectively. then press `+` or `-` to increase or decrease the value.

- `addLength()` in `Spring.java`

# **[Version 1.1.4]** - 2023-09-07

### Added

- `getter methods` in `ReadOnlySoftBody.java`: new methods to replace the ones placed in `ReadOnlyModel.java`

- `debugging stats panel` in `View.java`: added a panel that displays debugging information about the softbodies.

- `hard limit` for mass changes in `SoftBody.java`: added a hard limit for mass changes so that the mass of a softbody can't be less than 0.5.

### Removed

- `getter methods` in `ReadOnlyModel.java`: removed the methods that were moved to `ReadOnlySoftBody.java`

# **[Version 1.1.5]** - 2023-09-08

### Changed

- `massChange`, `pressureChange`, `springLengthChange` & `springConstantChange` in `SoftBody.java`: Changed the order of their indicator keys. documented in `README.md`.

- `applyFriction()` in `SoftBody.java`: friction used to be applied to velocity and force, after some testing, force changes did nothing. Not sure why but I'd guess its because force was reset every tick. Now friction is only applied to velocity.

- `getId()` in `SoftBodyModel.java`: changed the method to return the id of the softbody, instead of the controller returning it to view. This is because the model should be sending the data to the view, not the controller.

### Tested

- `accumulateCollisionForce()` in `SoftBody.java`: Tested the possibility of handling softbody collisions by accumulating forces instead of changing the positions of the points and setting velocity to 0 when they collide with a surface. I hypothesized that this will be more accurate, since force is being reflected but there was some issues with it so the idea was scrapped.

# **[Version 1.1.6]** - 2023-09-13 - Improved MVC Implementation

### Added

- `various graphical methods` in `ReadOnlyModel.java`: added methods that return graphical booleans,this way view can use getters instead of accessing the graphical booleans statically.

- `Draw Softbody Info Toggle`: added a toggle that controls whether or not the softbody info is drawn, via the `2` key.

### Changed

- `Method names` in `Rectangle.java`: Changed the names to more accurately represent what they do.

- `keyPressed()` and `keyReleased()` in `SoftBodyController.java`: Changed the methods to more accurately represent MVC relationships.

### Moved

- `Event Methods` in `SoftBody.java` to `SoftBodyController.java`: Moved the event methods to `SoftBodyController.java` since the controller is the one that should be handling the events. This is a step towards a more accurate MVC implementation.

### Removed

- `input flags` in `SoftBodyController.java`: Removed the input flags since they are already being set when a key is pressed directly to the model by the controller.

- `keyPressed()` and `keyReleased()` in `SoftBody.java`: Removed the methods since they are now exclusively handled in `SoftBodyController.java`

# **[Version 1.2.0]** - 2023-09-16 - Polygon Collisions

### Added

- `Polygon2D.java`: Added a class that represents a polygon with double precision.

- `translate()` in `Point2D.java` & `Rectangle.java`: Added a method that translates the point or rectangle by a given x and y value.

- `ReadOnlyPolygon2D` class: Added a class that represents a read only polygon with double precision.

- `getReadOnlyPolygons()` in `ReadOnlyModel` & implemented in `SoftBodyModel.java`: Added a method that returns a list of read only polygons that represent the softbodies.

- `default constructor` in `Rectangle`

- `handleBoundaryCollision()` in `SoftBody.java`: Added a method that handles collisions with polygons/ obstacles.

### Changed

- `var names` in `SoftBody.java`: Changed the names of some variables that used to have final signatures.

# **[Version 1.2.1]** - 2023-09-16 - Polygon Refactoring

### Added

- `SoftBodyHandler` class: Added a class that handles the updating of softbodies.
- `BouncePad` class: Added a class that represents a bounce pad which is a subclass of `Polygon2D`.
- `getSoftBodyHandlers()` in `SoftBodyModel.java`: Added a method that returns a list of softbody handlers.

### Removed

- `SoftBodyCode.md`: removed this file, the current implementation is likely permanent.

### Changed

- `List<Point2D> points` in `Polygon2D.java` to `List<MassPoint> points`: Changed the type of the list to `MassPoint` so that subclasses of the polygon class can be moved around, and have momentum/ carry force like softbodies.
- `Polygon2D` is now an abstract class.

### Moved

- `Polygon2D` to `polygon` package: Moved the class to the polygon package to support expansion into sub classes of polygon.

- `All Handler Methods` in `SoftBody` to `SoftBodyHandler.java`: Moved all the handler/ updater methods to the handler class.

# **[Version 1.2.2]** - 2023-09-22

## This Patch requires heavy refactoring

### Added

- `Point selection`: select points on the selected body by scrolling up or down.

- `Fixed Points`: points can now be fixed. Open the softbody info panel by pressing 2, which will also display the selected point, press 3 and the selected point will be fixed. Press 3 again to toggle.

- `Body` Class: Added a class that represents a body. Abstarcted a lot of the code from `SoftBody.java` into this class. Still needs to be refactored heavily.

- `Regular & Pressurized SoftBody` Classes: added classes that extend Body, which represent SoftBodies and PressurizedSoftBodies respectively.

### Removed

- `Physical Calculation Methods` in `SoftBody.java`: removed the class since it is now replaced by the `SoftBodyHandler` class.

### Changed

- `SoftBodyHandler`: Changed the class in anticipation of a non pressurized softbody implementation.

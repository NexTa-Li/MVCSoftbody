# Change Log

## [Version 0.0.1] - 2023-09-02

### Changed

- `Debugging Configuration`: Changed the project from debugging config so that it runs as intended via Driver.java

## [Version 0.0.2] - 2023-09-02

### Removed

- `Artifacts` in `checkCollisions()` & `handleSoftbodyCollisions()`: Removing some code artifacts that were left over from old ideas/ debugging.

## [Version 0.0.3] - 2023-09-02

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

## [Version 1.0.0] - 2023-09-04

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
  . This explains why it was more difficult to push a body from the right side to the left, because its bounding box was always smaller on the right side. so a maxBounds x position of 100.99 would be truncated to
  100, which means if an edge was flat past the bounding box it would be impossible to push the body to the left. This is no longer the case. To make sure this definitely isn't still happening I could also add a slight margin to the bounding box, but I don't think it's necessary. Anyways this was bothering me forever so I'm very excited that this is finally fixed.

### TODO:

- `Test` that the new bounding box is working as intended.

- `Test` that the new `isMerged()` method is working as intended.

- `Test` that `closestPointOnLineSegment()` in `SoftBodyUtil.java` is working as intended.

- `Change` the display implementation so that `view` handles all the drawing instead of telling each softbody to draw itself.

- Use the new `isMerged()` method to prevent softbodies from getting trapped inside each other

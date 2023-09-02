# Change Log

## [Version 0.0.1] - 2023-09-02

### Changed

- `Debugging Configuration`: Changed the project from debugging config so that it runs as intended via Driver.java

## [Version 0.0.2] - 2023-09-02

### Removed

- `Artifacts` in `checkCollisions()` & `handleSoftbodyCollisions()`: Removing some code artifacts that were left over from old ideas/ debugging.

## [Version 0.0.3] - 2023-09-02

### Changed

#### Softbody.java

- `integrateHuen()`: Temporarily removed friction from the integration, while debugging.

- `handleSoftBodyCollisions()`: Uncommented the code that changed the positions of the two points on the edge that was collided with. This was commented out for debugging purposes. This code will most likely be removed in the future.

### Fixed

- `checkCollisions()` in `SoftBody.java`: Fixed a bug with the ray casting algorithm that caused it to miss collisions where the ray was cast through a point. When a ray was cast through a point, the algorithm would count an intersection for both edges of the point, causing the algorithm to miss the collision.

### Removed

- `accumulateSoftBodyCollisionForce()`: Removed this method. It may be added back in the future but for now an implementation that doesn't require it is being explored.

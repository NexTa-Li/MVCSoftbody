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

# Testing handleSoftBodyCollisions Options

```java
    void handleSoftBodyCollisions(int i) {
        for (int j = 0; j < softBodies.size(); j++) {

            // skip self collision
            if (softBodies.get(j) == this)
                continue;

            // skip if theres no collision
            this.collided = false;
            if (!checkCollision(points.get(i).getPosition(), softBodies.get(j)))
                continue;

            // skip if this point is at the same position as the closest point
            if (points.get(i).getPosition().equals(closestPoint))
                continue;

            int p1 = edgePointIndices[0];
            int p2 = edgePointIndices[1];

            Vector2D n = new Vector2D(points.get(i).getPosition().getX() - closestPoint.getX(), // normal Vector
                    points.get(i).getPosition().getY() - closestPoint.getY());

            n.normalize();
            points.get(i).setPosition(closestPoint.getX(), closestPoint.getY());

            // velocity of this point
            double vx = points.get(i).getVelocityX();
            double vy = points.get(i).getVelocityY();

            // other bodies point velocity
            double o_vx = softBodies.get(j).points.get(p1).getVelocityX();
            double o_vy = softBodies.get(j).points.get(p1).getVelocityY();

            // handle vertex collisions
            if (p2 == -1) {
                double p = 2 * ((vx * n.getX() + vy * n.getY()) - (o_vx * n.getX() + o_vy * n.getY()))
                        / (SOFTBODY_MASS + softBodies.get(j).SOFTBODY_MASS);

                double vx1 = vx - (p * SOFTBODY_MASS * n.getX());
                double vy1 = vy - (p * SOFTBODY_MASS * n.getY());

                double vx2 = o_vx + (p * softBodies.get(j).SOFTBODY_MASS * n.getX());
                double vy2 = o_vy + (p * softBodies.get(j).SOFTBODY_MASS * n.getY());

                points.get(i).setVelocity(vx1, vy1);
                softBodies.get(j).points.get(p1).setVelocity(vx2, vy2);

                continue;
            }

            // else handle edge collisions
            double edgeVx = (o_vx + softBodies.get(j).points.get(p2).getVelocityX()) / 2.0;
            double edgeVy = (o_vy + softBodies.get(j).points.get(p2).getVelocityY()) / 2.0;

            double p = 2 * ((vx * n.getX() + vy * n.getY()) - (edgeVx * n.getX() + edgeVy * n.getY()))
                    / (SOFTBODY_MASS + softBodies.get(j).SOFTBODY_MASS);

            double vx1 = vx - (p * SOFTBODY_MASS * n.getX());
            double vy1 = vy - (p * SOFTBODY_MASS * n.getY());

            double vx2 = edgeVx + (p * softBodies.get(j).SOFTBODY_MASS * n.getX());
            double vy2 = edgeVy + (p * softBodies.get(j).SOFTBODY_MASS * n.getY());

            points.get(i).setVelocity(vx1, vy1);
            softBodies.get(j).points.get(p1).setVelocity(vx2, vy2);
            softBodies.get(j).points.get(p2).setVelocity(vx2, vy2);
        }
    }

```

### New option (w/ testers)

```java

void handleSoftBodyCollisions(int i) {
        for (int j = 0; j < softBodies.size(); j++) {

            // skip self collision
            if (softBodies.get(j) == this) {
                continue;
            }

            // skip if theres no collision
            this.collided = false;
            if (!checkCollision(points.get(i).getPosition(), softBodies.get(j))) {
                continue;
            }

            // skip if this point is at the same position as the closest point
            if (points.get(i).getPosition().equals(closestPoint)) {
                continue;
            }

            int p1 = edgePointIndices[0];
            int p2 = edgePointIndices[1];

            // normal Vector
            Vector2D n = new Vector2D(points.get(i).getPosition().getX() - closestPoint.getX(),
                    points.get(i).getPosition().getY() - closestPoint.getY());

            n.normalize();

            // Calculate movement amounts
            double moveAmount = n.getLength() * 0.5; // distance from closest point to the edge
            double length = n.getLength();
            // velocity of this point
            double vx = points.get(i).getVelocityX();
            double vy = points.get(i).getVelocityY();

            // other bodies point velocity
            double o_vx = softBodies.get(j).points.get(p1).getVelocityX();
            double o_vy = softBodies.get(j).points.get(p1).getVelocityY();

            // handle concave collisions where closest point is a vertex (masspoint)
            if (p2 == -1) {

                points.get(i).setPosition(closestPoint.getX(), closestPoint.getY());

                // calculate impulse
                double p = 2 * ((vx * n.getX() + vy * n.getY()) - (o_vx * n.getX() + o_vy *
                        n.getY()))
                        / (SOFTBODY_MASS + softBodies.get(j).SOFTBODY_MASS);

                double vx1 = vx - (p * softBodies.get(j).SOFTBODY_MASS * n.getX());
                double vy1 = vy - (p * softBodies.get(j).SOFTBODY_MASS * n.getY());

                double vx2 = o_vx + (p * SOFTBODY_MASS * n.getX());
                double vy2 = o_vy + (p * SOFTBODY_MASS * n.getY());

                vx1 *= ModelConfig.BOUNCINESS;
                vy1 *= ModelConfig.BOUNCINESS;
                vx2 *= ModelConfig.BOUNCINESS;
                vy2 *= ModelConfig.BOUNCINESS;

                points.get(i).setVelocity(vx1, vy1);
                softBodies.get(j).points.get(p1).setVelocity(vx2, vy2);

                continue;
            }

            // Calculate distances from points to the closest point
            double distP1 = points.get(p1).getPosition().distance(closestPoint);
            double distP2 = points.get(p2).getPosition().distance(closestPoint);

            // Calculate edge length
            double totalDist = points.get(p1).getPosition().distance(points.get(p2).getPosition());

            if (totalDist == 0) // extremely rare case where both points are on the same spot (avoid / 0.0)
                continue;

            // Calculate movement factors based on distances
            double moveFactorP1 = distP1 / totalDist;
            double moveFactorP2 = distP2 / totalDist;

            n.multiply(moveAmount);

            // Calculate new positions for each point
            double newX1 = points.get(i).getPositionX() - n.getX();
            double newY1 = points.get(i).getPositionY() - n.getY();

            n.normalize();
            n.multiply(moveFactorP1 * length);
            // System.out.println(moveFactorP1);
            double newX2 = softBodies.get(j).points.get(p1).getPositionX() + n.getX();
            double newY2 = softBodies.get(j).points.get(p1).getPositionY() + n.getY();

            n.normalize();
            n.multiply(moveFactorP2 * length);
            // System.out.println(moveFactorP2);
            double newX3 = softBodies.get(j).points.get(p2).getPositionX() + n.getX();
            double newY3 = softBodies.get(j).points.get(p2).getPositionY() + n.getY();

            // Calculate the new positions for the edge points using the closestToSingle
            // vector

            // Update positions
            // System.out.println("Handling collision with: " + j);
            // System.out.println("oldx1: " + points.get(i).getPositionX() + " oldy1: " +
            // points.get(i).getPositionY());
            points.get(i).setPosition(newX1, newY1);

            // System.out.println("newX1: " + newX1 + " newY1: " + newY1);

            // System.out.println("p1: " + softBodies.get(j).points.get(p1).getPositionX() +
            // ", "
            // + softBodies.get(j).points.get(p1).getPositionY());
            // System.out.println("p2: " + softBodies.get(j).points.get(p2).getPositionX() +
            // ", "
            // + softBodies.get(j).points.get(p2).getPositionY());

            // System.out.println("Closest Point: " + closestPoint.getX() + ", " +
            // closestPoint.getY());
            // System.out.println("newX2: " + newX2 + " newY2: " + newY2 + " newX3: " +
            // newX3 + " newY3: " + newY3);

            // softBodies.get(j).points.get(p1).setPosition(newX2, newY2);
            // softBodies.get(j).points.get(p2).setPosition(newX3, newY3);

            n.normalize();

            // else handle edge collisions
            double edgeVx = (o_vx + softBodies.get(j).points.get(p2).getVelocityX()) / 2.0;
            double edgeVy = (o_vy + softBodies.get(j).points.get(p2).getVelocityY()) / 2.0;

            double p = 2 * ((vx * n.getX() + vy * n.getY()) - (edgeVx * n.getX() + edgeVy * n.getY()))
                    / (SOFTBODY_MASS + softBodies.get(j).SOFTBODY_MASS);

            double vx1 = vx - (p * SOFTBODY_MASS * n.getX());
            double vy1 = vy - (p * SOFTBODY_MASS * n.getY());

            double vx2 = edgeVx + (p * softBodies.get(j).SOFTBODY_MASS * n.getX());
            double vy2 = edgeVy + (p * softBodies.get(j).SOFTBODY_MASS * n.getY());

            vx1 *= ModelConfig.BOUNCINESS;
            vy1 *= ModelConfig.BOUNCINESS;
            vx2 *= ModelConfig.BOUNCINESS;
            vy2 *= ModelConfig.BOUNCINESS;

            points.get(i).setVelocity(vx1, vy1);
            softBodies.get(j).points.get(p1).setVelocity(vx2, vy2);
            softBodies.get(j).points.get(p2).setVelocity(vx2, vy2);
        }
    }
```

### New option (w/out testers)

```java

void handleSoftBodyCollisions(int i) {
        for (int j = 0; j < softBodies.size(); j++) {

            // skip self collision
            if (softBodies.get(j) == this) {
                continue;
            }

            // skip if theres no collision
            this.collided = false;
            if (!checkCollision(points.get(i).getPosition(), softBodies.get(j))) {
                continue;
            }

            // skip if this point is at the same position as the closest point
            if (points.get(i).getPosition().equals(closestPoint)) {
                continue;
            }

            int p1 = edgePointIndices[0];
            int p2 = edgePointIndices[1];

            // normal Vector
            Vector2D n = new Vector2D(points.get(i).getPosition().getX() - closestPoint.getX(),
                    points.get(i).getPosition().getY() - closestPoint.getY());

            n.normalize();

            // Calculate movement amounts
            double moveAmount = n.getLength() * 0.5; // distance from closest point to the edge
            double length = n.getLength();
            // velocity of this point
            double vx = points.get(i).getVelocityX();
            double vy = points.get(i).getVelocityY();

            // other bodies point velocity
            double o_vx = softBodies.get(j).points.get(p1).getVelocityX();
            double o_vy = softBodies.get(j).points.get(p1).getVelocityY();

            // handle concave collisions where closest point is a vertex (masspoint)
            if (p2 == -1) {
                points.get(i).setPosition(closestPoint.getX(), closestPoint.getY());

                // calculate impulse
                double p = 2 * ((vx * n.getX() + vy * n.getY()) - (o_vx * n.getX() + o_vy *
                        n.getY()))
                        / (SOFTBODY_MASS + softBodies.get(j).SOFTBODY_MASS);

                double vx1 = vx - (p * softBodies.get(j).SOFTBODY_MASS * n.getX());
                double vy1 = vy - (p * softBodies.get(j).SOFTBODY_MASS * n.getY());

                double vx2 = o_vx + (p * SOFTBODY_MASS * n.getX());
                double vy2 = o_vy + (p * SOFTBODY_MASS * n.getY());

                points.get(i).setVelocity(vx1, vy1);
                softBodies.get(j).points.get(p1).setVelocity(vx2, vy2);

                continue;
            }

            // Calculate distances from points to the closest point
            double distP1 = points.get(p1).getPosition().distance(closestPoint);
            double distP2 = points.get(p2).getPosition().distance(closestPoint);

            // Calculate edge length
            double totalDist = points.get(p1).getPosition().distance(points.get(p2).getPosition());

            if (totalDist == 0) // extremely rare case where both points are on the same spot (avoid / 0.0)
                continue;

            // Calculate movement factors based on distances
            double moveFactorP1 = distP1 / totalDist;
            double moveFactorP2 = distP2 / totalDist;

            n.multiply(moveAmount);

            // Calculate new positions for each point
            double newX1 = points.get(i).getPositionX() - n.getX();
            double newY1 = points.get(i).getPositionY() - n.getY();

            n.normalize();
            n.multiply(moveFactorP1 * length);
            double newX2 = softBodies.get(j).points.get(p1).getPositionX() + n.getX();
            double newY2 = softBodies.get(j).points.get(p1).getPositionY() + n.getY();

            n.normalize();
            n.multiply(moveFactorP2 * length);
            double newX3 = softBodies.get(j).points.get(p2).getPositionX() + n.getX();
            double newY3 = softBodies.get(j).points.get(p2).getPositionY() + n.getY();

            // Update positions
            points.get(i).setPosition(newX1, newY1);
            softBodies.get(j).points.get(p1).setPosition(newX2, newY2);
            softBodies.get(j).points.get(p2).setPosition(newX3, newY3);

            n.normalize();

            // else handle edge collisions
            double edgeVx = (o_vx + softBodies.get(j).points.get(p2).getVelocityX()) / 2.0;
            double edgeVy = (o_vy + softBodies.get(j).points.get(p2).getVelocityY()) / 2.0;

            double p = 2 * ((vx * n.getX() + vy * n.getY()) - (edgeVx * n.getX() + edgeVy * n.getY()))
                    / (SOFTBODY_MASS + softBodies.get(j).SOFTBODY_MASS);

            double vx1 = vx - (p * SOFTBODY_MASS * n.getX());
            double vy1 = vy - (p * SOFTBODY_MASS * n.getY());

            double vx2 = edgeVx + (p * softBodies.get(j).SOFTBODY_MASS * n.getX());
            double vy2 = edgeVy + (p * softBodies.get(j).SOFTBODY_MASS * n.getY());

            points.get(i).setVelocity(vx1, vy1);
            softBodies.get(j).points.get(p1).setVelocity(vx2, vy2);
            softBodies.get(j).points.get(p2).setVelocity(vx2, vy2);
        }
    }
```

### accumulateSoftBodyCollisionForce (Force based collision resolution)

```java

    void accumulateSoftBodyCollisionForce() {
        for (int i = 0; i < points.size(); i++) {
            for (int j = 0; j < softBodies.size(); j++) {

                // skip self collision
                if (softBodies.get(j) == this) {
                    continue;
                }

                // skip if theres no collision
                this.collided = false;
                if (!checkCollision(points.get(i).getPosition(), softBodies.get(j))) {
                    continue;
                }

                // skip if this point is at the same position as the closest point
                if (points.get(i).getPosition().equals(closestPoint)) {
                    continue;
                }

                int p1 = edgePointIndices[0];
                int p2 = edgePointIndices[1];

                // normal Vector
                Vector2D n = new Vector2D(points.get(i).getPositionX() - closestPoint.getX(),
                        points.get(i).getPositionY() - closestPoint.getY());

                n.normalize();

                // Calculate movement amounts
                double moveAmount = n.getLength(); // distance from closest point to the edge

                // velocity of this point
                double fx = points.get(i).getForceX();
                double fy = points.get(i).getForceY();

                // other bodies point force
                double o_fx = softBodies.get(j).points.get(p1).getForceX();
                double o_fy = softBodies.get(j).points.get(p1).getForceY();

                // handle vertex collisions
                if (p2 == -1) {
                    System.out.println("Handling vertex collision with: " + j);
                    points.get(i).setPosition(closestPoint.getX(), closestPoint.getY());

                    // calculate impulse
                    double p = 2 * ((fx * n.getX() + fy * n.getY()) - (o_fx * n.getX() + o_fy *
                            n.getY()))
                            / (SOFTBODY_MASS + softBodies.get(j).SOFTBODY_MASS);

                    double fx1 = fx - (p * softBodies.get(j).SOFTBODY_MASS * n.getX());
                    double fy1 = fy - (p * softBodies.get(j).SOFTBODY_MASS * n.getY());

                    double fx2 = o_fx + (p * SOFTBODY_MASS * n.getX());
                    double fy2 = o_fy + (p * SOFTBODY_MASS * n.getY());

                    points.get(i).addForce(fx1, fy1);
                    softBodies.get(j).points.get(p1).addForce(fx2, fy2);

                    continue;
                }

                // Calculate edge length
                double totalDist = points.get(p1).getPosition().distance(points.get(p2).getPosition());

                if (totalDist == 0) // extremely rare case where all points are at the same position
                    continue;

                n.multiply(moveAmount);
                // Calculate new positions for each point
                double newX1 = points.get(i).getPositionX() - n.getX();
                double newY1 = points.get(i).getPositionY() - n.getY();

                points.get(i).setPosition(newX1, newY1);
                n.normalize();

                // else handle edge collisions
                double edgeFx = (o_fx + softBodies.get(j).points.get(p2).getForceX()) / 2.0;
                double edgeFy = (o_fy + softBodies.get(j).points.get(p2).getForceY()) / 2.0;

                double p = 2 * ((fx * n.getX() + fy * n.getY()) - (edgeFx * n.getX() + edgeFy * n.getY()))
                        / (SOFTBODY_MASS + softBodies.get(j).SOFTBODY_MASS);

                double fx1 = fx - (p * SOFTBODY_MASS * n.getX());
                double fy1 = fy - (p * SOFTBODY_MASS * n.getY());

                double fx2 = edgeFx + (p * softBodies.get(j).SOFTBODY_MASS * n.getX());
                double fy2 = edgeFy + (p * softBodies.get(j).SOFTBODY_MASS * n.getY());

                points.get(i).addForce(fx1, fy1);
                softBodies.get(j).points.get(p1).addForce(fx2, fy2);
                softBodies.get(j).points.get(p2).addForce(fx2, fy2);
            }
        }
    }

```

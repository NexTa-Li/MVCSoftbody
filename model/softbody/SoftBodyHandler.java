package model.softbody;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;

import model.ModelConfig;
import model.SoftBodyModel;
import model.geometry.Point2D;
import model.geometry.Vector2D;
import model.masspoint.MassPoint;
import model.polygon.Polygon2D;
import view.ViewConfig;

public class SoftBodyHandler {
    private SoftBody body;
    private SoftBodyModel model;

    private List<SoftBody> softBodies;
    private List<Polygon2D> polygons;

    public boolean keyUp, keyDown, keyLeft, keyRight;
    public boolean increase, decrease;
    public boolean pressureChange, springLengthChange, springConstantChange, massChange;

    int[] edgePointIndices;
    Point2D closestPoint = new Point2D();

    double xMin, xMax, yMin, yMax;

    public SoftBodyHandler(SoftBody softBody, SoftBodyModel model) {
        this.model = model;
        this.body = softBody;

        edgePointIndices = new int[3]; // find a better soln

        this.softBodies = model.softBodies;
        this.polygons = model.polygons;
    }

    /**
     * calls the method that accumulates forces for this tick, then calls the motion
     * integration method.
     * 
     * <p>
     * If the body.pressure of the softbody is less than the final body.pressure,
     * the body.pressure
     * is increased by this.FINAL_PRESSURE / ModelConfig.FILL_DURATION each tick.
     * This allows for the user to define how long it takes for the softbody to
     * inflate to its final body.pressure, which also means that the longer the
     * duration,
     * the more stable the softbody will be during and after the pressurization
     * period.
     * </p>
     */
    public void idle() {

        accumulateForces();
        integrateHuen();

        // fill in body.pressure
        if (body.pressure < body.targetPressure) {
            body.pressure += (body.targetPressure / ModelConfig.FILL_DURATION);
        }
    }

    /**
     * Accumulates all forces acting on the softbody.
     */
    void accumulateForces() {

        accumulateGravityForce();
        accumulateSpringForce();
        accumulatePressureForce();
    }

    /**
     * Method to accumulate both User force and gravity force.
     * These two steps can be merged together into a single iteration over the
     * body.points list.
     * 
     * <p>
     * First sets the body.points gravity to 0 and then adds the
     * SoftBodyModel.gravity to
     * each point. Then adds the user force to each point if the corresponding key
     * is pressed.
     * 
     * <br>
     * If the body.pressure of the softbody is less than the final body.pressure,
     * the gravity
     * force is not added to the body.points.
     * </p>
     */
    void accumulateGravityForce() {

        // Pressurization period
        if (body.pressure < body.targetPressure) {
            for (MassPoint p : body.points) {
                p.getForce().set(0.0, 0.0);
            }
            return;
        }

        if (massChange) {
            if (increase) {
                body.mass = body.mass >= 3.5 ? 3.5 : body.mass + body.mass / 100.0;
            }
            if (decrease) {
                body.mass = body.mass <= 0.5 ? 0.5 : body.mass - body.mass / 100.0;
            }
        }

        for (MassPoint massPoint : body.points) {
            massPoint.setForce(SoftBodyModel.gravity.getX(), SoftBodyModel.gravity.getY());
            massPoint.getForce().multiply(body.mass); // F = m * a

            if (keyUp) {
                massPoint.addForceY(-ModelConfig.USER_FORCE * body.mass);
            }
            if (keyDown) {
                massPoint.addForceY(ModelConfig.USER_FORCE * body.mass);
            }
            if (keyLeft) {
                massPoint.addForceX(-ModelConfig.USER_FORCE * body.mass);
            }
            if (keyRight) {
                massPoint.addForceX(ModelConfig.USER_FORCE * body.mass);
            }
        }
    }

    /**
     * Method to accumulate spring force.
     * 
     * <p>
     * Iterates through every spring in the body.springs list and calculates the
     * force.
     * </p>
     * 
     * @see Notes.md for more information on the calculations
     */
    void accumulateSpringForce() {
        Vector2D forceVector = new Vector2D(0.0, 0.0);
        int p1, p2;
        double x1, x2, y1, y2, distance, force, velocityX, velocityY;

        if (springConstantChange) {
            if (increase) {
                body.springConstant += body.springConstant / 100.0;
            }
            if (decrease) {
                body.springConstant -= body.springConstant / 100.0;
            }
        }

        for (int i = 0; i < body.springs.size(); i++) {
            p1 = body.springs.get(i).getP1();
            p2 = body.springs.get(i).getP2();

            x1 = body.points.get(p1).getPositionX();
            x2 = body.points.get(p2).getPositionX();
            y1 = body.points.get(p1).getPositionY();
            y2 = body.points.get(p2).getPositionY();

            // calculate distance, This is current length of spring
            distance = body.points.get(p1).getPosition().distance(body.points.get(p2).getPosition());

            // calculate force
            if (distance != 0) { // skip 0 to avoid division by 0
                velocityX = body.points.get(p1).getVelocityX() - body.points.get(p2).getVelocityX();
                velocityY = body.points.get(p1).getVelocityY() - body.points.get(p2).getVelocityY();

                force = (distance - body.springs.get(i).getLength()) * body.springConstant +
                        (velocityX * (x1 - x2) + velocityY * (y1 - y2)) * body.SPRING_DAMPING / distance;

                // calculate force vector
                forceVector.setX(force * ((x1 - x2) / distance));
                forceVector.setY(force * ((y1 - y2) / distance));

                // add force to body.points
                body.points.get(p1).subtractForce(forceVector);
                body.points.get(p2).addForce(forceVector);

                // calculate normal vector ~
                // (perpendicular to spring, used for body.pressure calculations)
                body.springs.get(i).setNormalVector(((y2 - y1) / distance), ((x1 - x2) / distance));
            }

            if (springLengthChange) {
                if (increase) {
                    body.springs.get(i).addLength(body.springs.get(i).getLength() / 100.0);
                }
                if (decrease) {
                    body.springs.get(i).addLength(-body.springs.get(i).getLength() / 100.0);
                }
            }
        }
    }

    void accumulatePressureForce() {
        int p1, p2;
        double x1, x2, y1, y2, distance, volume = 0.0;

        if (pressureChange) {
            if (increase) {
                body.targetPressure += body.targetPressure / 100.0;
            }
            if (decrease) {
                body.targetPressure -= body.targetPressure / 100.0;
            }
            body.pressure = body.targetPressure;
        }

        for (int i = 0; i < body.springs.size(); i++) {
            p1 = body.springs.get(i).getP1();
            p2 = body.springs.get(i).getP2();

            x1 = body.points.get(p1).getPositionX();
            x2 = body.points.get(p2).getPositionX();
            y1 = body.points.get(p1).getPositionY();
            y2 = body.points.get(p2).getPositionY();

            // calculate distance
            distance = body.points.get(p1).getPosition().distance(body.points.get(p2).getPosition());

            // calculate volume
            double vT1 = (Math.abs(x1 - x2) * Math.abs(body.springs.get(i).getNormalVectorX()) * distance) / 2.0;
            double vT2 = (Math.abs(y1 - y2) * Math.abs(body.springs.get(i).getNormalVectorY()) * distance) / 2.0;

            volume += (vT1 + vT2) / 2.0;
        }

        // avoid division by 0
        volume = volume == 0.0 ? 1.0 : volume;

        // calculate body.pressure force
        for (int i = 0; i < body.NUM_POINTS; i++) {
            p1 = body.springs.get(i).getP1();
            p2 = body.springs.get(i).getP2();

            // calculate distance
            distance = body.points.get(p1).getPosition().distance(body.points.get(p2).getPosition());

            double pressureVolume = distance * body.pressure * (1.0 / volume);

            double p1fx = body.springs.get(i).getNormalVectorX() * pressureVolume;
            double p1fy = body.springs.get(i).getNormalVectorY() * pressureVolume;

            double p2fx = body.springs.get(i).getNormalVectorX() * pressureVolume;
            double p2fy = body.springs.get(i).getNormalVectorY() * pressureVolume;

            body.points.get(p1).addForce(p1fx, p1fy);
            body.points.get(p2).addForce(p2fx, p2fy);
        }
    }

    /**
     * Motion integration method.
     * Huens Predictor-Corrector (2nd order) Integration.
     * 
     * https://en.wikipedia.org/wiki/Heun%27s_method
     */
    void integrateHuen() {

        double deltaRotationY, deltaRotationX;

        ArrayList<Vector2D> ForceSaved = new ArrayList<Vector2D>();
        ArrayList<Vector2D> VelocitySaved = new ArrayList<Vector2D>();

        // reset bounding box
        xMin = Double.MAX_VALUE;
        xMax = Double.MIN_VALUE;
        yMin = Double.MAX_VALUE;
        yMax = Double.MIN_VALUE;

        for (int i = 0; i < body.NUM_POINTS; i++) {
            // handle friction
            applyFriction(i);
            ForceSaved.add(new Vector2D(body.points.get(i).getForce()));
            VelocitySaved.add(new Vector2D(body.points.get(i).getVelocity()));

            double dvx = body.points.get(i).getForceX() / body.mass * ModelConfig.timestep;
            double dvy = body.points.get(i).getForceY() / body.mass * ModelConfig.timestep;

            body.points.get(i).getVelocity().add(dvx, dvy);

            deltaRotationX = body.points.get(i).getVelocityX() * ModelConfig.timestep;
            deltaRotationY = body.points.get(i).getVelocityY() * ModelConfig.timestep;

            body.points.get(i).addPosition(deltaRotationX, deltaRotationY);

            xMin = Math.min(xMin, body.points.get(i).getPositionX());
            xMax = Math.max(xMax, body.points.get(i).getPositionX());
            yMin = Math.min(yMin, body.points.get(i).getPositionY());
            yMax = Math.max(yMax, body.points.get(i).getPositionY());
        }
        // assign the bounding box values
        body.boundingBox.x = xMin;
        body.boundingBox.y = yMin;
        body.boundingBox.width = xMax - xMin;
        body.boundingBox.height = yMax - yMin;

        accumulateForces();

        for (int i = 0; i < body.NUM_POINTS; i++) {

            double dvx = (body.points.get(i).getForceX() + ForceSaved.get(i).getX()) / body.mass * ModelConfig.timestep;
            double dvy = (body.points.get(i).getForceY() + ForceSaved.get(i).getY()) / body.mass * ModelConfig.timestep;

            body.points.get(i).setVelocityX(VelocitySaved.get(i).getX() + dvx / 2.0);
            body.points.get(i).setVelocityY(VelocitySaved.get(i).getY() + dvy / 2.0);

            deltaRotationX = body.points.get(i).getVelocityX() * ModelConfig.timestep;
            deltaRotationY = body.points.get(i).getVelocityY() * ModelConfig.timestep;

            body.points.get(i).addPosition(deltaRotationX, deltaRotationY);

            // keep track of min and max values for bounding box

            if (body.pressure == body.targetPressure) {
                handleSoftBodyCollision(i);
                handlePointCollision(i);
                handlePolygonCollision(i);
            }
            handleRigidCollision(i);
        }

    }

    /**
     * checks the body.points collided field, if true, apply friction. This is done
     * by
     * multiplying its velocity by 1.0 - the friction coefficient defined in
     * ModelConfig
     * 
     * @param i index of the point
     */
    void applyFriction(int i) {
        if (body.points.get(i).isCollided()) {
            // scale the velocity down (1.0 represents no friction i.e full velo retention)
            body.points.get(i).getVelocity().multiply(1.0 - ModelConfig.SURFACE_FRICTION_COEFFICIENT);
            body.points.get(i).setCollided(false);
            return;
        }
    }

    /**
     * Checks for and Resolves collisions between the point at index i and another
     * softbody
     * 
     * @param i index of the point
     */
    void handleSoftBodyCollision(int i) {
        for (int j = 0; j < softBodies.size(); j++) {

            // skip self collision
            if (softBodies.get(j) == body) {
                continue;
            }

            // skip if theres no collision
            if (!checkCollision(body.points.get(i).getPosition(), softBodies.get(j))) {
                continue;
            }
            // System.out.println("Collision detected with: " + j);

            // skip if this point is at the same position as the closest point
            if (body.points.get(i).getPosition().equals(closestPoint)) {
                continue;
            }

            int p1 = edgePointIndices[0];
            int p2 = edgePointIndices[1];

            // normal Vector
            Vector2D n = new Vector2D(body.points.get(i).getPositionX() - closestPoint.getX(),
                    body.points.get(i).getPositionY() - closestPoint.getY());

            // velocity of this point
            double vx = body.points.get(i).getVelocityX();
            double vy = body.points.get(i).getVelocityY();

            // other bodies point velocity
            double o_vx = softBodies.get(j).points.get(p1).getVelocityX();
            double o_vy = softBodies.get(j).points.get(p1).getVelocityY();

            n.divide(2.0);

            // Calculate new positions for each point
            double newX1 = closestPoint.getX() + n.getX();
            double newY1 = closestPoint.getY() + n.getY();

            double newX2 = softBodies.get(j).points.get(p1).getPositionX() + n.getX();
            double newY2 = softBodies.get(j).points.get(p1).getPositionY() + n.getY();

            // handle vertex collisions
            if (p2 == -1) {
                // System.out.println("Handling vertex collision with: " + j);

                body.points.get(i).setPosition(newX1, newY1);
                softBodies.get(j).points.get(p1).setPosition(newX2, newY2);

                n.normalize();
                // calculate impulse
                double p = 2 * ((vx * n.getX() + vy * n.getY()) - (o_vx * n.getX() + o_vy *
                        n.getY()))
                        / (body.mass + softBodies.get(j).mass);

                double vx1 = vx - (p * softBodies.get(j).mass * n.getX());
                double vy1 = vy - (p * softBodies.get(j).mass * n.getY());

                double vx2 = o_vx + (p * body.mass * n.getX());
                double vy2 = o_vy + (p * body.mass * n.getY());

                vx1 *= ModelConfig.BOUNCINESS;
                vy1 *= ModelConfig.BOUNCINESS;
                vx2 *= ModelConfig.BOUNCINESS;
                vy2 *= ModelConfig.BOUNCINESS;

                body.points.get(i).setVelocity(vx1, vy1);
                softBodies.get(j).points.get(p1).setVelocity(vx2, vy2);

                continue; // skip the rest
            }

            double newX3 = softBodies.get(j).points.get(p2).getPositionX() + n.getX();
            double newY3 = softBodies.get(j).points.get(p2).getPositionY() + n.getY();

            // Calculate the new positions for the edge body.points using the
            // closestToSingle
            // vector

            body.points.get(i).setPosition(newX1, newY1);

            softBodies.get(j).points.get(p1).setPosition(newX2, newY2);
            softBodies.get(j).points.get(p2).setPosition(newX3, newY3);

            // normalize for the collision calculations
            n.normalize();

            // handle edge collisions
            double edgeVx = (o_vx + softBodies.get(j).points.get(p2).getVelocityX()) / 2.0;
            double edgeVy = (o_vy + softBodies.get(j).points.get(p2).getVelocityY()) / 2.0;

            double p = 2 * ((vx * n.getX() + vy * n.getY()) - (edgeVx * n.getX() + edgeVy * n.getY()))
                    / (body.mass + softBodies.get(j).mass);

            double vx1 = vx - (p * softBodies.get(j).mass * n.getX());
            double vy1 = vy - (p * softBodies.get(j).mass * n.getY());

            double vx2 = edgeVx + (p * body.mass * n.getX());
            double vy2 = edgeVy + (p * body.mass * n.getY());

            vx1 *= ModelConfig.BOUNCINESS;
            vy1 *= ModelConfig.BOUNCINESS;
            vx2 *= ModelConfig.BOUNCINESS;
            vy2 *= ModelConfig.BOUNCINESS;

            body.points.get(i).setVelocity(vx1, vy1);
            softBodies.get(j).points.get(p1).setVelocity(vx2, vy2);
            softBodies.get(j).points.get(p2).setVelocity(vx2, vy2);

        }
    }

    /**
     * Collisions with surfaces (ground, ceiling, walls)
     * 
     * If the point is outside the surface, move it back inside
     * and set the velocity to 0
     * 
     * @param i index of the point
     */
    void handleRigidCollision(int i) {
        // collision with ground
        if (body.points.get(i).getPosition().getY() >= ViewConfig.PANEL_HEIGHT - ModelConfig.WALL_THICKNESS) {
            body.points.get(i).getPosition().setY(ViewConfig.PANEL_HEIGHT - ModelConfig.WALL_THICKNESS);
            body.points.get(i).getVelocity().setY(0);

            // set body.points as collided
            body.points.get(i).setCollided(true);
        }
        // collision with ceiling
        if (body.points.get(i).getPosition().getY() <= 0 + ModelConfig.WALL_THICKNESS) {
            body.points.get(i).getPosition().setY(ModelConfig.WALL_THICKNESS);
            body.points.get(i).getVelocity().setY(0);

            body.points.get(i).setCollided(true);
        }

        // collision with walls
        if (body.points.get(i).getPosition().getX() >= ViewConfig.PANEL_WIDTH - ModelConfig.WALL_THICKNESS) {
            body.points.get(i).getPosition().setX(ViewConfig.PANEL_WIDTH - ModelConfig.WALL_THICKNESS);
            body.points.get(i).getVelocity().setX(0);

            body.points.get(i).setCollided(true);
        }

        if (body.points.get(i).getPosition().getX() < ModelConfig.WALL_THICKNESS) {
            body.points.get(i).getPosition().setX(ModelConfig.WALL_THICKNESS);
            body.points.get(i).getVelocity().setX(0);

            body.points.get(i).setCollided(true);
        }
    }

    /**
     * Handle collisions between two body.points on a softbody
     * 
     * If the distance between two body.points is less than the radius of a point,
     * then move the body.points apart.
     * 
     * @param i index of the point
     */
    void handlePointCollision(int i) {
        for (int j = 0; j < body.NUM_POINTS; j++) {

            if (i == j) {
                continue;
            }

            double distance = body.points.get(i).getPosition().distance(body.points.get(j).getPosition());

            if (distance >= ModelConfig.MASS_POINT_RADIUS || distance == 0) {
                continue;
            }

            // find the vector between the two body.points
            int p1 = i;
            int p2 = j;

            double x1 = body.points.get(p1).getPositionX();
            double x2 = body.points.get(p2).getPositionX();

            double y1 = body.points.get(p1).getPositionY();
            double y2 = body.points.get(p2).getPositionY();

            Vector2D collisionVector = new Vector2D(x2 - x1, y2 - y1);
            collisionVector.divide(distance);

            // normalize the vector
            collisionVector.normalize();
            Vector2D normalVector = new Vector2D(collisionVector);

            // take velocities into account
            double p = 2 * (body.points.get(p1).getVelocityX() * normalVector.getX()
                    + body.points.get(p1).getVelocityY() * normalVector.getY()
                    - body.points.get(p2).getVelocityX() * normalVector.getX()
                    + body.points.get(p2).getVelocityY() * normalVector.getY())
                    / (body.mass + body.mass);

            double vx1 = body.points.get(p1).getVelocityX() - p * body.mass * normalVector.getX();
            double vy1 = body.points.get(p1).getVelocityY() - p * body.mass * normalVector.getY();
            double vx2 = body.points.get(p2).getVelocityX() + p * body.mass * normalVector.getX();
            double vy2 = body.points.get(p2).getVelocityY() + p * body.mass * normalVector.getY();

            body.points.get(p1).setVelocity(vx1, vy1);
            body.points.get(p2).setVelocity(vx2, vy2);
        }
    }

    void handlePolygonCollision(int i) {
        for (int j = 0; j < polygons.size(); j++) {

            // skip if theres no collision
            if (!checkCollision(body.points.get(i).getPosition(), polygons.get(j))) {
                continue;
            }

            // skip if this point is at the same position as the closest point
            if (body.points.get(i).getPosition().equals(closestPoint)) {
                continue;
            }

            Vector2D n = new Vector2D(body.points.get(i).getPositionX() - closestPoint.getX(),
                    body.points.get(i).getPositionY() - closestPoint.getY());

            // // velocity of this point
            double vx = body.points.get(i).getVelocityX();
            double vy = body.points.get(i).getVelocityY();

            // // handle vertex collisions
            // if (p2 == -1) {
            // // System.out.println("Handling vertex collision with: " + j);

            // body.points.get(i).setPosition(newX1, newY1);
            // softBodies.get(j).body.points.get(p1).setPosition(newX2, newY2);

            // n.normalize();
            // // calculate impulse
            // double p = 2 * ((vx * n.getX() + vy * n.getY()) - (o_vx * n.getX() + o_vy *
            // n.getY()))
            // / (body.mass + softBodies.get(j).body.mass);

            // double vx1 = vx - (p * softBodies.get(j).body.mass * n.getX());
            // double vy1 = vy - (p * softBodies.get(j).body.mass * n.getY());

            // double vx2 = o_vx + (p * body.mass * n.getX());
            // double vy2 = o_vy + (p * body.mass * n.getY());

            // vx1 *= ModelConfig.BOUNCINESS;
            // vy1 *= ModelConfig.BOUNCINESS;
            // vx2 *= ModelConfig.BOUNCINESS;
            // vy2 *= ModelConfig.BOUNCINESS;

            // body.points.get(i).setVelocity(vx1, vy1);
            // softBodies.get(j).body.points.get(p1).setVelocity(vx2, vy2);

            // continue; // skip the rest
            // }

            // double newX3 = softBodies.get(j).body.points.get(p2).getPositionX() +
            // n.getX();
            // double newY3 = softBodies.get(j).body.points.get(p2).getPositionY() +
            // n.getY();

            // // Calculate the new positions for the edge body.points using the
            // closestToSingle
            // // vector

            body.points.get(i).setPosition(closestPoint);

            // softBodies.get(j).body.points.get(p1).setPosition(newX2, newY2);
            // softBodies.get(j).body.points.get(p2).setPosition(newX3, newY3);

            // // normalize for the collision calculations
            // n.normalize();

            // // handle edge collisions
            // double edgeVx = (o_vx + softBodies.get(j).body.points.get(p2).getVelocityX())
            // /
            // 2.0;
            // double edgeVy = (o_vy + softBodies.get(j).body.points.get(p2).getVelocityY())
            // /
            // 2.0;

            double p = 2 * ((body.points.get(i).getVelocity().getX() * n.getX()
                    + body.points.get(i).getVelocity().getY() * n.getY())
                    - (0 * n.getX() + 0 * n.getY())) / (body.mass + 1);

            double vx1 = vx - (p * softBodies.get(j).mass * n.getX());
            double vy1 = vy - (p * softBodies.get(j).mass * n.getY());

            body.points.get(i).setVelocity(vx1, vy1);

        }
    }

    /**
     * check if the point is inside the soft body
     * 
     * Checks if the point is inside the bounding box of the soft body,
     * then checks if the point is inside the soft body.
     * 
     * First a horizontal ray is casted to the point, then we count the number of
     * intersections between the ray and the edges of the soft body. if the number
     * of intersections is odd, then the point is inside the soft body.
     * 
     * @param position the position of the point to check
     * @param other    the soft body to check against
     * @return true if the point is inside the soft body, false otherwise
     */
    public boolean checkCollision(Point2D position, SoftBody other) {
        // Check if the point is inside the bounding box of the other soft body
        if (!other.boundingBox.contains(position.getX(), position.getY())) {
            return false;
        }

        Line2D ray = new Line2D.Double(position.getX(), 0, position.getX(), position.getY());

        int points = other.points.size();
        int intersectionCount = 0;

        // set the closest point to the first point in the list
        closestPoint = new Point2D(other.points.get(0).getPosition());

        for (int i = 0; i < points; i++) {
            Point2D edgeStart = other.points.get(i).getPosition();
            Point2D edgeEnd = other.points.get((i + 1) % points).getPosition();

            Point2D tempClosestPoint = SoftBodyUtil.closestPointOnLineSegment(position, edgeStart, edgeEnd);

            double tempDistance = position.distance(tempClosestPoint);
            double oldDistance = position.distance(closestPoint);

            // check if the closest point is on an edge and closer than the previous closest
            if (tempDistance <= oldDistance) {
                edgePointIndices[0] = i;
                edgePointIndices[1] = tempClosestPoint.equals(edgeStart) ? -1 : (i + 1) % points;
                // this should probably only be set if theres actually a collision
                closestPoint.setLocation(tempClosestPoint.getX(), tempClosestPoint.getY());
            }

            /*
             * check if the ray intersects the edge.
             * Also check if the ray is at the same height as a point on
             * the edge, if it is, then the next edge will be intersected, so we skip this
             */
            if (!SoftBodyUtil.checkIntersection(ray, edgeStart, edgeEnd)) {
                continue;
            }

            if (position.getY() != edgeEnd.getY()) {
                intersectionCount++;
            }
        }

        // If the number of intersections is odd, then the point is inside the soft body
        return intersectionCount % 2 == 1;
    }

    public boolean checkCollision(Point2D position, Polygon2D other) {

        // Check if the point is inside the bounding box
        if (!other.contains(position.getX(), position.getY())) {
            return false;
        }

        int points = other.npoints;
        closestPoint = new Point2D(other.points.get(0).getPosition());
        for (int i = 0; i < points; i++) {
            Point2D edgeStart = other.points.get(i).getPosition();
            Point2D edgeEnd = other.points.get((i + 1) % points).getPosition();

            Point2D tempClosestPoint = SoftBodyUtil.closestPointOnLineSegment(position, edgeStart, edgeEnd);

            double tempDistance = position.distance(tempClosestPoint);
            double oldDistance = position.distance(closestPoint);

            // check if the closest point is on an edge and closer than the previous closest
            if (tempDistance <= oldDistance) {
                // this should probably only be set if theres actually a collision
                closestPoint.setLocation(tempClosestPoint.getX(), tempClosestPoint.getY());
            }
        }

        return true;
    }

    public void resetChangeVars() {
        massChange = false;
        pressureChange = false;
        springLengthChange = false;
        springConstantChange = false;
    }

}

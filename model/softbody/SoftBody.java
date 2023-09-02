package model.softbody;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.event.KeyEvent;
import java.util.ArrayList;
import java.util.List;

import model.ModelConfig;
import model.SoftBodyModel;
import model.Spring;
import model.geometry.Point2D;
import model.geometry.Vector2D;
import model.masspoint.MassPoint;
import model.masspoint.ReadOnlyMassPoint;
import view.ViewConfig;

public class SoftBody implements ReadOnlySoftBody {
    List<MassPoint> points;
    List<Spring> springs;
    List<SoftBody> softBodies;

    final int NUM_POINTS;
    final double SOFTBODY_MASS;
    final double SPRING_CONSTANT;
    final double SPRING_DAMPING;
    final double FINAL_PRESSURE;

    int[] edgePointIndices;

    double pressure = 0.0;
    Point2D closestPoint = new Point2D();
    boolean collided = false;

    Rectangle boundingBox;

    double xMin, xMax, yMin, yMax;

    boolean keyUp = false;
    boolean keyDown = false;
    boolean keyLeft = false;
    boolean keyRight = false;

    /*
     * Maintaining invariants
     * 
     * Checks if the position, force or velocity of a point is NaN. If it is, exit
     * the program. this way we can find out where the NaN is coming from.
     * 
     * NaN values will always cause the softbody to explode and dissappear.
     * 
     * Modify later if you add other objects that change state.
     */
    private void checkNaN(int i, String location) {
        if (Double.isNaN(points.get(i).getPosition().getX())) {
            System.out.println("NaN detected " + location);
            System.out.println("Position NaN");
            System.exit(0);
        }
        if (Double.isNaN(points.get(i).getForce().getX())) {
            System.out.println("NaN detected " + location);
            System.out.println("Force NaN");
            System.exit(0);
        }
        if (Double.isNaN(points.get(i).getVelocity().getX())) {
            System.out.println("NaN detected " + location);
            System.out.println("Velocity NaN");
            System.exit(0);
        }
    }

    /**
     * Creates a Softbody object with the specified parameters.
     * 
     * @param x          x position of the center of the softbody
     * @param y          y position of the center of the softbody
     * @param numPoints  number of points in the softbody
     * @param radius     radius of the softbody
     * @param mass       mass of each point in the softbody
     * @param ks         spring constant
     * @param kd         spring damping
     * @param pressure   Final Pressure of the softbody
     * @param softBodies reference to a list of all other softbodies
     */
    public SoftBody(double x, double y, int numPoints, double radius, double mass, double ks, double kd,
            double pressure, List<SoftBody> softBodies) {
        this.NUM_POINTS = numPoints;
        this.SOFTBODY_MASS = mass;
        this.SPRING_CONSTANT = ks;
        this.SPRING_DAMPING = kd;
        this.FINAL_PRESSURE = pressure;

        points = new ArrayList<MassPoint>();
        springs = new ArrayList<Spring>();
        this.softBodies = softBodies; // pass a reference to the list of softbodies

        edgePointIndices = new int[3]; // find a better soln

        boundingBox = new Rectangle((int) x, (int) y, (int) radius * 2, (int) radius * 2); // find a better soln

        createSoftBody(x, y, radius);
    }

    /**
     * Creates the points and springs of the softbody.
     * 
     * <p>
     * This method is called by the constructor.
     * The points are created in a circle around the center of the softbody, where
     * radius determines how far away the points are from the center.
     * 
     * <br>
     * The springs are created between each point and the next point in the list.
     * The index of each point is also added to the spring object, so that the
     * spring can access the points in the list.
     * </p>
     * 
     * @param x      x position of the center of the softbody
     * @param y      y position of the center of the softbody
     * @param radius radius of the softbody
     */
    void createSoftBody(double x, double y, double radius) {

        for (int i = 0; i < NUM_POINTS; i++) {
            Point2D position = new Point2D();
            position.setX(radius * Math.cos(2 * Math.PI * i / NUM_POINTS) + x);
            position.setY(radius * Math.sin(2 * Math.PI * i / NUM_POINTS) + y);
            points.add(new MassPoint(position));
        }

        // create springs
        for (int i = 0; i < NUM_POINTS - 1; i++) {
            addSpring(i, i + 1);
        }
        addSpring(NUM_POINTS - 1, 0);
    }

    /**
     * Adds springs between the points at the specified indexes in the points list
     * to the springs list.
     * 
     * length is calculated using the distance between the two points, which means
     * that the radius of the softbody
     * actually does have an impact because it causes the spring to have a certain
     * equilibrium length, which is used to calculate spring force.
     *
     * @param point1 index of the first point
     * @param point2 index of the second point
     */
    void addSpring(int point1, int point2) {
        // calculate length
        double length = Math.sqrt(points.get(point1).getPosition().distance(points.get(point2).getPosition()));

        this.springs.add(new Spring(point1, point2, length));
    }

    /**
     * Accumulates all forces acting on the softbody.
     */
    void accumulateForces() {

        accumulateGravityForce();
        accumulateSpringForce();
        accumulatePressureForce();
        // accumulateSoftBodyCollisionForce();
    }

    /**
     * calls the method that accumulates forces for this tick, then calls the motion
     * integration method.
     * 
     * <p>
     * If the pressure of the softbody is less than the final pressure, the pressure
     * is increased by this.FINAL_PRESSURE / ModelConfig.FILL_DURATION each tick.
     * This allows for the user to define how long it takes for the softbody to
     * inflate to its final pressure, which also means that the longer the duration,
     * the more stable the softbody will be during and after the pressurization
     * period.
     * </p>
     */
    public void idle() {

        accumulateForces();
        integrateHuen();

        // fill in pressure
        if (pressure < FINAL_PRESSURE) {
            pressure += (FINAL_PRESSURE / ModelConfig.FILL_DURATION);
        }
    }

    /**
     * Method to accumulate both User force and gravity force.
     * These two steps can be merged together into a single iteration over the
     * points list.
     * 
     * <p>
     * First sets the points gravity to 0 and then adds the SoftBodyModel.gravity to
     * each point. Then adds the user force to each point if the corresponding key
     * is pressed.
     * 
     * <br>
     * If the pressure of the softbody is less than the final pressure, the gravity
     * force is not added to the points.
     * </p>
     */
    void accumulateGravityForce() {

        // Pressurization period
        if (pressure < FINAL_PRESSURE) {
            for (MassPoint p : points) {
                p.getForce().set(0.0, 0.0);
            }
            return;
        }

        for (MassPoint massPoint : points) {
            massPoint.setForce(SoftBodyModel.gravity.getX(), SoftBodyModel.gravity.getY());
            massPoint.getForce().multiply(SOFTBODY_MASS); // F = m * a

            if (keyUp) {
                massPoint.addForceY(-ModelConfig.USER_FORCE * SOFTBODY_MASS);
            }
            if (keyDown) {
                massPoint.addForceY(ModelConfig.USER_FORCE * SOFTBODY_MASS);
            }
            if (keyLeft) {
                massPoint.addForceX(-ModelConfig.USER_FORCE * SOFTBODY_MASS);
            }
            if (keyRight) {
                massPoint.addForceX(ModelConfig.USER_FORCE * SOFTBODY_MASS);
            }
        }
    }

    /**
     * Method to accumulate spring force.
     * 
     * <p>
     * Iterates through every spring in the springs list and calculates the
     * force.
     * </p>
     * 
     * @see Notes.md for more information on the calculations
     */
    void accumulateSpringForce() {
        Vector2D forceVector = new Vector2D(0.0, 0.0);
        int p1, p2;
        double x1, x2, y1, y2, distance, force, velocityX, velocityY;

        for (int i = 0; i < springs.size(); i++) {
            p1 = springs.get(i).getP1();
            p2 = springs.get(i).getP2();

            x1 = points.get(p1).getPositionX();
            x2 = points.get(p2).getPositionX();
            y1 = points.get(p1).getPositionY();
            y2 = points.get(p2).getPositionY();

            // calculate distance, This is current length of spring
            distance = points.get(p1).getPosition().distance(points.get(p2).getPosition());

            // calculate force
            if (distance != 0) { // skip 0 to avoid division by 0
                velocityX = points.get(p1).getVelocityX() - points.get(p2).getVelocityX();
                velocityY = points.get(p1).getVelocityY() - points.get(p2).getVelocityY();

                force = (distance - springs.get(i).getLength()) * SPRING_CONSTANT +
                        (velocityX * (x1 - x2) + velocityY * (y1 - y2)) * SPRING_DAMPING / distance;

                // calculate force vector
                forceVector.setX(force * ((x1 - x2) / distance));
                forceVector.setY(force * ((y1 - y2) / distance));

                // add force to points
                points.get(p1).subtractForce(forceVector);
                points.get(p2).addForce(forceVector);

                // calculate normal vector ~
                // (perpendicular to spring, used for pressure calculations)
                springs.get(i).setNormalVector(((y2 - y1) / distance), ((x1 - x2) / distance));
            }
        }
    }

    void accumulatePressureForce() {
        int p1, p2;
        double x1, x2, y1, y2, distance, volume = 0.0;

        for (int i = 0; i < springs.size(); i++) {
            p1 = springs.get(i).getP1();
            p2 = springs.get(i).getP2();

            x1 = points.get(p1).getPositionX();
            x2 = points.get(p2).getPositionX();
            y1 = points.get(p1).getPositionY();
            y2 = points.get(p2).getPositionY();

            // calculate distance
            distance = points.get(p1).getPosition().distance(points.get(p2).getPosition());

            // calculate volume
            double vT1 = (Math.abs(x1 - x2) * Math.abs(springs.get(i).getNormalVectorX()) * distance) / 2.0;
            double vT2 = (Math.abs(y1 - y2) * Math.abs(springs.get(i).getNormalVectorY()) * distance) / 2.0;

            volume += (vT1 + vT2) / 2.0;
        }

        // avoid division by 0
        volume = volume == 0.0 ? 1.0 : volume;

        // calculate pressure force
        for (int i = 0; i < NUM_POINTS; i++) {
            p1 = springs.get(i).getP1();
            p2 = springs.get(i).getP2();

            // calculate distance
            distance = points.get(p1).getPosition().distance(points.get(p2).getPosition());

            double pressureVolume = distance * pressure * (1.0 / volume);

            double p1fx = springs.get(i).getNormalVectorX() * pressureVolume;
            double p1fy = springs.get(i).getNormalVectorY() * pressureVolume;

            double p2fx = springs.get(i).getNormalVectorX() * pressureVolume;
            double p2fy = springs.get(i).getNormalVectorY() * pressureVolume;

            points.get(p1).addForce(p1fx, p1fy);
            points.get(p2).addForce(p2fx, p2fy);

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

        // bounding box
        xMin = points.get(0).getPosition().getX();
        xMax = points.get(0).getPosition().getX();
        yMin = points.get(0).getPosition().getY();
        yMax = points.get(0).getPosition().getY();

        for (int i = 0; i < NUM_POINTS; i++) {
            // handle friction
            applyFriction(i);

            ForceSaved.add(new Vector2D(points.get(i).getForce()));
            VelocitySaved.add(new Vector2D(points.get(i).getVelocity()));

            double dvx = points.get(i).getForceX() / SOFTBODY_MASS * ModelConfig.timestep;
            double dvy = points.get(i).getForceY() / SOFTBODY_MASS * ModelConfig.timestep;

            points.get(i).getVelocity().add(dvx, dvy);

            deltaRotationX = points.get(i).getVelocityX() * ModelConfig.timestep;
            deltaRotationY = points.get(i).getVelocityY() * ModelConfig.timestep;

            points.get(i).addPosition(deltaRotationX, deltaRotationY);

            // keep track of min and max values for bounding box
            xMin = Math.min(xMin, points.get(i).getPositionX());
            xMax = Math.max(xMax, points.get(i).getPositionX());
            yMin = Math.min(yMin, points.get(i).getPositionY());
            yMax = Math.max(yMax, points.get(i).getPositionY());

            handleSoftBodyCollisions(i);
            handleRigidCollisions(i);
            handlePointCollisions(i);
        }

        // assign the bounding box values
        boundingBox.x = (int) xMin;
        boundingBox.y = (int) yMin;
        boundingBox.width = (int) (xMax - xMin);
        boundingBox.height = (int) (yMax - yMin);

        accumulateForces();

        for (int i = 0; i < NUM_POINTS; i++) {
            applyFriction(i);
            double dvx = (points.get(i).getForceX() + ForceSaved.get(i).getX()) / SOFTBODY_MASS * ModelConfig.timestep;
            double dvy = (points.get(i).getForceY() + ForceSaved.get(i).getY()) / SOFTBODY_MASS * ModelConfig.timestep;

            points.get(i).setVelocityX(VelocitySaved.get(i).getX() + dvx / 2.0);
            points.get(i).setVelocityY(VelocitySaved.get(i).getY() + dvy / 2.0);

            deltaRotationX = points.get(i).getVelocityX() * ModelConfig.timestep;
            deltaRotationY = points.get(i).getVelocityY() * ModelConfig.timestep;

            points.get(i).addPosition(deltaRotationX, deltaRotationY);

            handleSoftBodyCollisions(i);
            handleRigidCollisions(i);
            handlePointCollisions(i);
        }
    }

    /**
     * checks the points collided field, if true, apply friction. This is done by
     * multiplying its velocity by 1.0 - the friction coefficient defined in
     * ModelConfig
     * 
     * @param i index of the point
     */
    void applyFriction(int i) {
        if (points.get(i).isCollided()) {
            // scale the velocity down (1.0 represents no friction i.e full velocity
            // retention)
            points.get(i).getVelocity().multiply(1.0 - ModelConfig.SURFACE_FRICTION_COEFFICIENT);
            points.get(i).getForce().multiply(1.0 - ModelConfig.SURFACE_FRICTION_COEFFICIENT);
            points.get(i).setCollided(false);
        }
    }

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

    /**
     * Checks for and Resolves collisions between the point at index i and another
     * softbody
     * 
     * @param i index of the point
     */
    void handleSoftBodyCollisions(int i) {
        for (int j = 0; j < softBodies.size(); j++) {

            // skip self collision
            if (softBodies.get(j) == this) {
                continue;
            }

            // skip if theres no collision
            this.collided = false; // fix
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
            double moveAmount = n.getLength() / 2.0; // distance from closest point to the edge

            // velocity of this point
            double vx = points.get(i).getVelocityX();
            double vy = points.get(i).getVelocityY();

            // other bodies point velocity
            double o_vx = softBodies.get(j).points.get(p1).getVelocityX();
            double o_vy = softBodies.get(j).points.get(p1).getVelocityY();

            // handle vertex collisions
            if (p2 == -1) {
                System.out.println("Handling vertex collision with: " + j);
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

            if (totalDist == 0) // extremely rare case where all points are at the same position
                continue; // skip because we can't divide by 0

            // Calculate movement factors based on distances
            double moveFactorP1 = distP2 / totalDist;
            double moveFactorP2 = distP1 / totalDist;

            n.multiply(moveAmount);

            // Calculate new positions for each point
            double newX1 = points.get(i).getPositionX() - n.getX();
            double newY1 = points.get(i).getPositionY() - n.getY();

            n.normalize();
            n.multiply(moveFactorP1 * moveAmount / 2.0);
            // System.out.println(moveFactorP1);
            double newX2 = softBodies.get(j).points.get(p1).getPositionX() + n.getX();
            double newY2 = softBodies.get(j).points.get(p1).getPositionY() + n.getY();

            n.normalize();
            n.multiply(moveFactorP2 * moveAmount / 2.0);
            // System.out.println(moveFactorP2);
            double newX3 = softBodies.get(j).points.get(p2).getPositionX() + n.getX();
            double newY3 = softBodies.get(j).points.get(p2).getPositionY() + n.getY();

            // Calculate the new positions for the edge points using the closestToSingle
            // vector

            // Update positions
            // System.out.println("Handling collision with: " + j);
            // System.out.println("Old Position: " + points.get(i).getPosition());
            points.get(i).setPosition(newX1, newY1);

            // System.out.println("New Position: " + points.get(i).getPosition());

            // System.out.println("old: " + softBodies.get(j).points.get(p1).getPosition() +
            // ", "
            // + softBodies.get(j).points.get(p2).getPosition());

            // System.out.println("Closest Point: " + closestPoint);

            // Without these lines the sim is much more stable
            // softBodies.get(j).points.get(p1).setPosition(newX2, newY2);
            // softBodies.get(j).points.get(p2).setPosition(newX3, newY3);

            // System.out.println("new points: " +
            // softBodies.get(j).points.get(p1).getPosition() + ", "
            // + softBodies.get(j).points.get(p2).getPosition());

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

    /**
     * Collisions with surfaces (ground, ceiling, walls)
     * 
     * If the point is outside the surface, move it back inside
     * and set the velocity to 0
     * 
     * @param i index of the point
     */
    void handleRigidCollisions(int i) {
        // collision with ground
        if (points.get(i).getPosition().getY() >= ViewConfig.PANEL_HEIGHT - ModelConfig.WALL_THICKNESS) {
            points.get(i).getPosition().setY(ViewConfig.PANEL_HEIGHT - ModelConfig.WALL_THICKNESS);
            points.get(i).getVelocity().setY(0);

            // set points as collided
            points.get(i).setCollided(true);
        }
        // collision with ceiling
        if (points.get(i).getPosition().getY() <= 0 + ModelConfig.WALL_THICKNESS) {
            points.get(i).getPosition().setY(ModelConfig.WALL_THICKNESS);
            points.get(i).getVelocity().setY(0);

            points.get(i).setCollided(true);
        }

        // collision with walls
        if (points.get(i).getPosition().getX() >= ViewConfig.PANEL_WIDTH - ModelConfig.WALL_THICKNESS) {
            points.get(i).getPosition().setX(ViewConfig.PANEL_WIDTH - ModelConfig.WALL_THICKNESS);
            points.get(i).getVelocity().setX(0);

            points.get(i).setCollided(true);
        }

        if (points.get(i).getPosition().getX() < ModelConfig.WALL_THICKNESS) {
            points.get(i).getPosition().setX(ModelConfig.WALL_THICKNESS);
            points.get(i).getVelocity().setX(0);

            points.get(i).setCollided(true);
        }
    }

    /**
     * Handle collisions between two points on a softbody
     * 
     * If the distance between two points is less than the radius of a point,
     * then move the points apart.
     * 
     * @param i index of the point
     */
    void handlePointCollisions(int i) {
        for (int j = 0; j < NUM_POINTS; j++) {

            if (i == j) {
                continue;
            }

            double distance = points.get(i).getPosition().distance(points.get(j).getPosition());

            if (distance >= ModelConfig.MASS_POINT_RADIUS) {
                continue;
            }

            // find the vector between the two points
            int p1 = i;
            int p2 = j;

            double x1 = points.get(p1).getPositionX();
            double x2 = points.get(p2).getPositionX();

            double y1 = points.get(p1).getPositionY();
            double y2 = points.get(p2).getPositionY();

            Vector2D collisionVector = new Vector2D(x2 - x1, y2 - y1);
            collisionVector.divide(distance);

            // normalize the vector
            collisionVector.normalize();
            Vector2D normalVector = new Vector2D(collisionVector);

            // take velocities into account
            double p = 2 * (points.get(p1).getVelocityX() * normalVector.getX()
                    + points.get(p1).getVelocityY() * normalVector.getY()
                    - points.get(p2).getVelocityX() * normalVector.getX()
                    + points.get(p2).getVelocityY() * normalVector.getY())
                    / (SOFTBODY_MASS + SOFTBODY_MASS);

            double vx1 = points.get(p1).getVelocityX() - p * SOFTBODY_MASS * normalVector.getX();
            double vy1 = points.get(p1).getVelocityY() - p * SOFTBODY_MASS * normalVector.getY();
            double vx2 = points.get(p2).getVelocityX() + p * SOFTBODY_MASS * normalVector.getX();
            double vy2 = points.get(p2).getVelocityY() + p * SOFTBODY_MASS * normalVector.getY();

            points.get(p1).setVelocity(vx1, vy1);
            points.get(p2).setVelocity(vx2, vy2);

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
     * This method will need to be reworked to full accomodate concave shapes since
     * ray casting will not work for concave shapes. This is important since the
     * soft body will be able to deform into concave shapes.
     * 
     * TODO: rework this method to work with concave shapes, and also rework the
     * solution for checking whether the closest point is a vertex or an edge
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

        Point2D rayStart = new Point2D(position.getX(), 0);

        int points = other.points.size();
        int intersectionCount = 0;

        for (int i = 0; i < points; i++) {
            Point2D edgeStart = new Point2D(other.points.get(i).getPosition());
            Point2D edgeEnd = new Point2D(other.points.get((i + 1) % points).getPosition());

            Point2D tempClosestPoint = SoftBodyUtil.closestPointOnLineSegment(position, edgeStart, edgeEnd);

            double tempDistance = position.distance(tempClosestPoint);
            double oldDistance = position.distance(closestPoint);

            // check if the closest point is on an edge and closer than the previous closest
            if (tempDistance <= oldDistance || !collided) {
                edgePointIndices[0] = i;
                edgePointIndices[1] = (i + 1) % points;

                // this should probably only be set if theres actually a collision
                closestPoint.setLocation(tempClosestPoint.getX(), tempClosestPoint.getY());
                collided = true;
            }

            // check if the ray intersects the edge
            if (SoftBodyUtil.checkIntersection(rayStart, position, edgeStart, edgeEnd)) {
                intersectionCount++;
            }
        }

        // If the number of intersections is odd, then the point is inside the soft body
        if (intersectionCount % 2 == 1) {

            // naive approach to check if the closest point is a vertex
            for (int i = 0; i < points; i++) {
                Point2D edgeStart = new Point2D(other.points.get(i).getPosition());

                double oldDistance = position.distance(closestPoint);
                double closestEdgePointDistance = position.distance(edgeStart);

                // check if the closest point is a vertex and closer than the previous closest
                if (closestEdgePointDistance < oldDistance) {
                    edgePointIndices[0] = i;
                    edgePointIndices[1] = -1;

                    // this should probably only be set if theres actually a collision
                    closestPoint.setLocation(edgeStart.getX(), edgeStart.getY());
                    System.out.println("Closest point is a vertex");
                }
            }
            return true;
        }
        // otherwise, the point is outside the soft body, so no collision
        collided = false;
        return false;
    }

    @Override
    public int getNumPoints() {
        return this.NUM_POINTS;
    }

    @Override
    public ArrayList<ReadOnlyMassPoint> getPoints() {
        return new ArrayList<>(this.points);
    }

    @Override
    public Rectangle getBoundingBox() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getBoundingBox'");
    }

    @Override
    public void paintComponent(Graphics graphics) {
        // draw softbody
        Graphics2D g = (Graphics2D) graphics;
        g.setStroke(new BasicStroke(2));

        g.setColor(Color.gray);
        // g.fillPolygon(getXArr(), getYArr(), NUM_POINTS);

        g.setColor(Color.gray);
        for (int i = 0; i < NUM_POINTS; i++) {
            g.drawLine((int) points.get(springs.get(i).getP1()).getPosition().getX(),
                    (int) points.get(springs.get(i).getP1()).getPosition().getY(),
                    (int) points.get(springs.get(i).getP2()).getPosition().getX(),
                    (int) points.get(springs.get(i).getP2()).getPosition().getY());
        }

        g.setColor(Color.red);

        for (int i = 0; i < NUM_POINTS; i++) {

            g.fillOval((int) (points.get(i).getPositionX() - ModelConfig.POINT_SIZE),
                    (int) (points.get(i).getPositionY() - ModelConfig.POINT_SIZE), (int) (2 * ModelConfig.POINT_SIZE),
                    (int) (2 * ModelConfig.POINT_SIZE));
        }

    }

    public void keyPressed(KeyEvent e) {

        if (e.getKeyCode() == KeyEvent.VK_UP)
            keyUp = true;
        if (e.getKeyCode() == KeyEvent.VK_DOWN)
            keyDown = true;
        if (e.getKeyCode() == KeyEvent.VK_LEFT)
            keyLeft = true;
        if (e.getKeyCode() == KeyEvent.VK_RIGHT)
            keyRight = true;
    }

    public void keyReleased(KeyEvent e) {

        if (e.getKeyCode() == KeyEvent.VK_UP)
            keyUp = false;
        if (e.getKeyCode() == KeyEvent.VK_DOWN)
            keyDown = false;
        if (e.getKeyCode() == KeyEvent.VK_LEFT)
            keyLeft = false;
        if (e.getKeyCode() == KeyEvent.VK_RIGHT)
            keyRight = false;
    }

    /**
     * Used for drawing the softbody
     * 
     * @return array of x coordinates
     */
    int[] getXArr() {
        int[] xArr = new int[NUM_POINTS];
        for (int i = 0; i < NUM_POINTS; i++) {
            double x = points.get(i).getPositionX();
            xArr[i] = (int) x;
        }
        return xArr;
    }

    /**
     * Used for drawing the softbody
     * 
     * @return array of y coordinates
     */
    int[] getYArr() {
        int[] yArr = new int[NUM_POINTS];
        for (int i = 0; i < NUM_POINTS; i++) {
            double y = points.get(i).getPositionY();
            yArr[i] = (int) y;
        }
        return yArr;
    }
}
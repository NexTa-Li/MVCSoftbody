package model.polygon;

import model.geometry.Point2D;

import model.ModelConfig;
import model.geometry.Vector2D;
import view.ViewConfig;

public class BouncePad extends Polygon2D {

    public boolean bounce;
    public final Vector2D BOUNCE;
    private int distance;
    public Vector2D velocity = new Vector2D();

    Point2D origin = new Point2D(0, 0);
    Point2D translation = new Point2D(0, 0);

    public BouncePad(double mass, Vector2D direction, int distance, int force) {
        super(mass);
        if (force > 5 || force <= 0) {
            throw new IllegalArgumentException("force must be greater than 0 and less than or equal to 5");
        }

        this.BOUNCE = new Vector2D(direction);
        this.BOUNCE.normalize();
        this.BOUNCE.multiply(force);
        this.distance = distance;
    }

    void accumulateUserForce() {

        for (int i = 0; i < npoints; i++) {

            if (bounce) {

                if (distance - origin.distance(translation) > 1) {
                    // System.out.println("bounce");
                    points.get(i).setForce(BOUNCE.getX(), BOUNCE.getY());
                    continue;
                }
                bounce = false;
                velocity.set(0, 0);
            }

            if (origin.distance(translation) > 0.05) {
                points.get(i).setForce(-1 * BOUNCE.getX(), -1 * BOUNCE.getY());
                continue;
            }

            double dx = origin.getX() - translation.getX();
            double dy = origin.getY() - translation.getY();

            points.get(i).addPosition(dx, dy);

            // when it reaches origin margin
            this.velocity.set(0, 0);
            points.get(i).setForce(0, 0);

            if (i == npoints - 1) {
                translation.setLocation(0, 0);
            }
        }
    }

    void handleRigidCollision() {
        for (int i = 0; i < npoints; i++) {

            // collision with floor
            if (points.get(i).getPosition().getY() >= ViewConfig.PANEL_HEIGHT - ModelConfig.WALL_THICKNESS) {
                points.get(i).getPosition().setY(ViewConfig.PANEL_HEIGHT - ModelConfig.WALL_THICKNESS);
                velocity.set(0, 0);

                bounce = false;
            }

            // collision with ceiling
            if (points.get(i).getPosition().getY() <= 0 + ModelConfig.WALL_THICKNESS) {
                points.get(i).getPosition().setY(ModelConfig.WALL_THICKNESS);
                velocity.set(0, 0);
                bounce = false;
            }

            // collision with walls
            if (points.get(i).getPosition().getX() >= ViewConfig.PANEL_WIDTH - ModelConfig.WALL_THICKNESS) {
                points.get(i).getPosition().setX(ViewConfig.PANEL_WIDTH - ModelConfig.WALL_THICKNESS);
                velocity.set(0, 0);
                bounce = false;
            }

            if (points.get(i).getPosition().getX() < ModelConfig.WALL_THICKNESS) {
                points.get(i).getPosition().setX(ModelConfig.WALL_THICKNESS);
                velocity.set(0, 0);
                bounce = false;
            }
        }
    }

    public void idle() {
        accumulateUserForce();
        handleRigidCollision();
        integrateEuler();
    }

    void integrateEuler() {
        // Calculate acceleration based on the force and mass
        double ax = points.get(0).getForceX() / mass;
        double ay = points.get(0).getForceY() / mass;

        // Update velocity using Euler's method
        velocity.add(ax * ModelConfig.timestep, ay * ModelConfig.timestep);

        // Update position
        this.translate(velocity.getX(), velocity.getY());
        translation.add(velocity.getX(), velocity.getY());

    }

    public void toggleBounce() {
        if (origin.distance(translation) == 0) {
            bounce = !bounce;
        }
    }
}

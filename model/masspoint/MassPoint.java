package model.masspoint;

import model.geometry.*;

public class MassPoint implements ReadOnlyMassPoint {
    Point2D position; // x, y (change to point2D later)
    Vector2D velocity; // vx, vy
    Vector2D force; // fx, fy (force accumulator)

    // Scuffed implementation, needs to be fixed
    public boolean isFixed = false;

    /**
     * whether the mass point is inside another object
     * (or was inside another object in the previous frame)
     */
    boolean collided = false;

    public MassPoint(Point2D position) {
        this.position = position;
        this.force = new Vector2D(0.0, 0.0);
        this.velocity = new Vector2D(0.0, 0.0);
    }

    public MassPoint(double x, double y) {
        this(new Point2D(x, y));
    }

    // copy constructor
    public MassPoint(MassPoint p) {
        this.position = new Point2D(p.getPosition());
        this.force = new Vector2D(p.getForce());
        this.velocity = new Vector2D(p.getVelocity());

    }

    public boolean isCollided() {
        return collided;
    }

    public void setCollided(boolean collided) {
        this.collided = collided;
    }

    public Point2D getPosition() {
        return isFixed ? new Point2D(position) : position; // not safe, sacrifice for performance
    }

    public double getPositionX() {
        return position.getX();
    }

    public double getPositionY() {
        return position.getY();
    }

    public void addPosition(double x, double y) {

        if (isFixed) {
            return;
        }

        this.position.addX(x);
        this.position.addY(y);
    }

    public void setPosition(Point2D position) {
        if (isFixed) {
            return;
        }
        this.position = position;
    }

    public void setPosition(double x, double y) {
        if (isFixed) {
            return;
        }
        this.position.setLocation(x, y);
    }

    public void setPositionX(double x) {
        if (isFixed) {
            return;
        }
        this.position.setX(x);
    }

    public void setPositionY(double y) {
        this.position.setY(y);
    }

    public Vector2D getVelocity() {
        return isFixed ? new Vector2D(velocity) : velocity;
    }

    public double getVelocityX() {
        return velocity.getX();
    }

    public double getVelocityY() {
        return velocity.getY();
    }

    public void setVelocity(Vector2D velocity) {
        if (isFixed) {
            return;
        }
        this.velocity = velocity;
    }

    public void setVelocity(double vx, double vy) {
        if (isFixed) {
            return;
        }
        this.velocity.set(vx, vy);
    }

    public void setVelocityX(Double vx) {
        if (isFixed) {
            return;
        }
        this.velocity.setX(vx);
    }

    public void setVelocityY(Double vy) {
        if (isFixed) {
            return;
        }
        this.velocity.setY(vy);
    }

    public Vector2D getForce() {
        return isFixed ? new Vector2D(force) : force;
    }

    public double getForceX() {
        return force.getX();
    }

    public double getForceY() {
        return force.getY();
    }

    public void setForce(Vector2D force) {
        if (isFixed) {
            return;
        }
        this.force = force;
    }

    public void setForce(double fx, double fy) {
        if (isFixed) {
            return;
        }
        this.force.set(fx, fy);
    }

    public void setForceX(double fx) {
        if (isFixed) {
            return;
        }
        this.force.setX(fx);
    }

    public void setForceY(double fy) {
        if (isFixed) {
            return;
        }
        this.force.setY(fy);
    }

    public void addForce(Vector2D force) {
        if (isFixed) {
            return;
        }
        this.force.add(force);
    }

    public void addForce(double fx, double fy) {
        if (isFixed) {
            return;
        }
        this.force.add(fx, fy);
    }

    public void addForceX(double fx) {
        if (isFixed) {
            return;
        }
        this.force.addX(fx);
    }

    public void addForceY(double fy) {
        if (isFixed) {
            return;
        }
        this.force.addY(fy);
    }

    public void subtractForce(Vector2D force) {
        if (isFixed) {
            return;
        }
        this.force.subtract(force);
    }

    public void subtractForceX(double fx) {
        if (isFixed) {
            return;
        }
        this.force.subtractX(fx);
    }

    public void subtractForceY(double fy) {
        if (isFixed) {
            return;
        }
        this.force.subtractY(fy);
    }
}

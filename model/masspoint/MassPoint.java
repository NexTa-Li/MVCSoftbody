package model.masspoint;

import model.geometry.*;

public class MassPoint implements ReadOnlyMassPoint {
    Point2D position; // x, y (change to point2D later)
    Vector2D velocity; // vx, vy
    Vector2D force; // fx, fy (force accumulator)

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
        return position; // not safe, sacrifice for performance
    }

    public Double getPositionX() {
        return position.getX();
    }

    public Double getPositionY() {
        return position.getY();
    }

    public void addPosition(double x, double y) {
        this.position.addX(x);
        this.position.addY(y);
    }

    public void setPosition(Point2D position) {
        this.position = position;
    }

    public void setPosition(double x, double y) {
        this.position.setLocation(x, y);
    }

    public void setPositionX(Double x) {
        this.position.setX(x);
    }

    public void setPositionY(Double y) {
        this.position.setY(y);
    }

    public Vector2D getVelocity() {
        return velocity;
    }

    public Double getVelocityX() {
        return velocity.getX();
    }

    public Double getVelocityY() {
        return velocity.getY();
    }

    public void setVelocity(Vector2D velocity) {
        this.velocity = velocity;
    }

    public void setVelocity(double vx, double vy) {
        this.velocity.set(vx, vy);
    }

    public void setVelocityX(Double vx) {
        this.velocity.setX(vx);
    }

    public void setVelocityY(Double vy) {
        this.velocity.setY(vy);
    }

    public Vector2D getForce() {
        return force;
    }

    public Double getForceX() {
        return force.getX();
    }

    public Double getForceY() {
        return force.getY();
    }

    public void setForce(Vector2D force) {
        this.force = force;
    }

    public void setForce(double fx, double fy) {
        this.force.set(fx, fy);
    }

    public void setForceX(double fx) {
        this.force.setX(fx);
    }

    public void setForceY(double fy) {
        this.force.setY(fy);
    }

    public void addForce(Vector2D force) {
        this.force.add(force);
    }

    public void addForce(double fx, double fy) {
        this.force.add(fx, fy);
    }

    public void addForceX(double fx) {
        this.force.addX(fx);
    }

    public void addForceY(double fy) {
        this.force.addY(fy);
    }

    public void subtractForce(Vector2D force) {
        this.force.subtract(force);
    }

    public void subtractForceX(double fx) {
        this.force.subtractX(fx);
    }

    public void subtractForceY(double fy) {
        this.force.subtractY(fy);
    }
}

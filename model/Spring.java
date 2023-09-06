package model;

import model.geometry.Vector2D;

/**
 * Spring class
 * 
 * <p>
 * this class stores the indices of its mass points instead of the mass points
 * to prevent too much memory usage.
 * 
 * if you want to take hit on used memory, its possible to add some optimization
 * by storing and updating a line segment between the mass points.
 * </p>
 */
public class Spring {
    int p1, p2; // point indices
    double length; // resting length
    Vector2D NormalVector = new Vector2D(0.0, 0.0);

    public Spring(int p1, int p2, double length) {
        this.p1 = p1;
        this.p2 = p2;
        this.length = length;
    }

    public int getP1() {
        return p1;
    }

    public int getP2() {
        return p2;
    }

    public double getLength() {
        return this.length;
    }

    public void addLength(double length) {
        this.length += length;
    }

    public Vector2D getNormalVector() {
        return new Vector2D(this.NormalVector);
    }

    public double getNormalVectorX() {
        return this.NormalVector.getX();
    }

    public double getNormalVectorY() {
        return this.NormalVector.getY();
    }

    public void setNormalVector(Vector2D normalVector) {
        setNormalVector(normalVector.getX(), normalVector.getY());
    }

    public void setNormalVector(double x, double y) {
        this.NormalVector.set(x, y);
    }
}

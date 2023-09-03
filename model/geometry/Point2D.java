package model.geometry;

import java.util.Objects;

public class Point2D {
    double x, y;

    public Point2D() {
        this.x = 0.0;
        this.y = 0.0;
    }

    public Point2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point2D(Point2D p) {
        this.x = p.x;
        this.y = p.y;
    }

    public void add(Point2D p) {
        this.x += p.x;
        this.y += p.y;
    }

    public void add(double x, double y) {
        this.x += x;
        this.y += y;
    }

    public void subtract(Point2D p) {
        this.x -= p.x;
        this.y -= p.y;
    }

    public void subtract(double x, double y) {
        this.x -= x;
        this.y -= y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void addX(double x) {
        this.x += x;
    }

    public void addY(double y) {
        this.y += y;
    }

    public static double distance(double x1, double y1, double x2, double y2) {
        x1 -= x2;
        y1 -= y2;
        return (x1 * x1 + y1 * y1);
    }

    public double distance(double px, double py) {
        double dx = px - getX();
        double dy = py - getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double distance(Point2D p) {
        return distance(p.getX(), p.getY());
    }

    public void setLocation(double x, double y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public boolean equals(Object o) {

        if (o == null)
            return false;

        if (!(o instanceof Point2D)) {
            return false;
        }

        Point2D p = (Point2D) o;
        return (p.getX() == this.x && p.getY() == this.y);
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }

    @Override
    public String toString() {
        return "(" + String.format("%,.10f", x) + ", " + String.format("%,.10f", y) + ")";
    }
}

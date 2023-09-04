package model.geometry;

public class Rectangle {
    public double x;
    public double y;
    public double width;
    public double height;

    public Rectangle(double x, double y, double width, double height) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
    }

    public boolean contains(double x, double y) {
        return (x >= this.x &&
                y >= this.y &&
                x <= this.x + this.width &&
                y <= this.y + this.height);
    }

    public double getCenterX() {
        return x + width / 2.0;
    }

    public double getCenterY() {
        return y + height / 2.0;
    }

    // methods to return doubles
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getWidth() {
        return width;
    }

    public double getHeight() {
        return height;
    }

    // methods to return ints for drawing purposes
    public int X() {
        return (int) x;
    }

    public int Y() {
        return (int) y;
    }

    public int width() {
        return (int) width;
    }

    public int height() {
        return (int) height;
    }
}
package model.softbody;

import java.util.ArrayList;

import model.Spring;
import model.geometry.Point2D;
import model.masspoint.MassPoint;

public class PressurizedSoftBody extends Body {
    final int NUM_POINTS;

    public PressurizedSoftBody(double x, double y, int numPoints, double radius, double mass,
            double ks, double kd, double targetPressure) {
        super(x, y, mass, ks, kd);

        this.NUM_POINTS = numPoints;
        this.targetPressure = targetPressure;

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

    @Override
    public double getPressure() {
        return this.pressure;
    }

    @Override
    public ArrayList<MassPoint> points() {
        return new ArrayList<>(this.points);
    }
}

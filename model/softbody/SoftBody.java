package model.softbody;

import java.util.ArrayList;
import java.util.List;

import model.Spring;
import model.geometry.Point2D;
import model.geometry.Rectangle;
import model.masspoint.MassPoint;
import model.masspoint.ReadOnlyMassPoint;

public class SoftBody implements ReadOnlySoftBody {
    List<MassPoint> points;
    List<Spring> springs;

    final int NUM_POINTS;
    final double SPRING_DAMPING;

    double mass;
    double springConstant;
    double targetPressure;
    double pressure = 0.0;

    Rectangle boundingBox;

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
    public SoftBody(double x, double y, int numPoints, double radius, double mass,
            double ks, double kd, double targetPressure) {
        this.NUM_POINTS = numPoints;
        this.mass = mass;
        this.springConstant = ks;
        this.SPRING_DAMPING = kd;
        this.targetPressure = targetPressure;

        points = new ArrayList<MassPoint>();
        springs = new ArrayList<Spring>();

        boundingBox = new Rectangle(x, y, 1, 1); // bounding box is at the centre of the softbody

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
    public int getNumPoints() {
        return this.NUM_POINTS;
    }

    @Override
    public ArrayList<ReadOnlyMassPoint> getPoints() {
        return new ArrayList<>(this.points);
    }

    @Override
    public Rectangle getBoundingBox() {
        return this.boundingBox; // Privacy sacrifice for performance
    }

    /**
     * Used for drawing the softbody
     * 
     * @return array of x coordinates
     */
    @Override
    public int[] getXArr() {
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
    @Override
    public int[] getYArr() {
        int[] yArr = new int[NUM_POINTS];
        for (int i = 0; i < NUM_POINTS; i++) {
            double y = points.get(i).getPositionY();
            yArr[i] = (int) y;
        }
        return yArr;
    }

    @Override
    public double getPressure() {
        return this.pressure;
    }

    @Override
    public double getSpringConstant() {
        return this.springConstant;
    }

    @Override
    public double getSpringDamping() {
        return this.SPRING_DAMPING;
    }

    @Override
    public double getSpringRestLength() {
        return this.springs.get(0).getLength();
    }

    @Override
    public double getMass() {
        return this.mass;
    }

}
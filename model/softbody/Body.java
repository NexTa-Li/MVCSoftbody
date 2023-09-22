package model.softbody;

import java.util.ArrayList;
import java.util.List;

import model.Spring;
import model.geometry.Rectangle;
import model.masspoint.MassPoint;
import model.masspoint.ReadOnlyMassPoint;

public abstract class Body implements ReadOnlySoftBody {
    List<MassPoint> points;
    List<Spring> springs;

    final double SPRING_DAMPING;

    // int numPoints;

    double mass;
    double springConstant;
    double pressure = 0.0;
    double targetPressure = 0.0;

    Rectangle boundingBox;

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
    public Body(double x, double y, double mass, double ks, double kd) {
        this.mass = mass;
        this.springConstant = ks;
        this.SPRING_DAMPING = kd;

        points = new ArrayList<MassPoint>();
        springs = new ArrayList<Spring>();

        this.boundingBox = new Rectangle(x, y, 1, 1);
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
        int[] xArr = new int[points.size()];
        for (int i = 0; i < points.size(); i++) {
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
        int[] yArr = new int[points.size()];
        for (int i = 0; i < points.size(); i++) {
            double y = points.get(i).getPositionY();
            yArr[i] = (int) y;
        }
        return yArr;
    }

    public ArrayList<MassPoint> points() {
        return new ArrayList<>();
    }

    @Override
    public double getPressure() {
        return 0.0;
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
    public ArrayList<Spring> getSprings() {
        return new ArrayList<>(this.springs);
    }

    @Override
    public double getMass() {
        return this.mass;
    }

}

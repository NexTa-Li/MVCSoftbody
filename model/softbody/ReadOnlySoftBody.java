package model.softbody;

import java.util.ArrayList;

import model.masspoint.ReadOnlyMassPoint;
import model.Spring;
import model.geometry.Rectangle;

public interface ReadOnlySoftBody {
    // Bare minimum for now

    ArrayList<ReadOnlyMassPoint> getPoints();

    ArrayList<Spring> getSprings();

    public Rectangle getBoundingBox();

    public double getPressure();

    public double getSpringConstant();

    public double getSpringDamping();

    public double getSpringRestLength();

    public double getMass();

    public int[] getXArr();

    public int[] getYArr();
}

package model;

import java.util.ArrayList;

import model.softbody.ReadOnlySoftBody;

public interface ReadOnlyModel {
    ArrayList<ReadOnlySoftBody> getReadOnlySoftBodies();

    public int getId();

    public double getPressure();

    public double getNumPoints();

    public double getSpringConstant();

    public double getSpringDamping();

    public double getMass();

    public double getRadius();
}

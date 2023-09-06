package model.softbody;

import java.util.ArrayList;

import model.masspoint.ReadOnlyMassPoint;
import model.geometry.Rectangle;

public interface ReadOnlySoftBody {
    // Bare minimum for now
    public int getNumPoints();

    ArrayList<ReadOnlyMassPoint> getPoints();

    public Rectangle getBoundingBox();

    public int[] getXArr();

    public int[] getYArr();
}

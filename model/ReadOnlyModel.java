package model;

import java.util.ArrayList;

import model.geometry.Vector2D;
import model.polygon.ReadOnlyPolygon2D;
import model.softbody.ReadOnlySoftBody;

public interface ReadOnlyModel {
    ArrayList<ReadOnlySoftBody> getReadOnlySoftBodies();

    ArrayList<ReadOnlyPolygon2D> getReadOnlyPolygons();

    public int getId();

    public int getSelectedPoint();

    public Vector2D getGravity();

    public boolean isBodyFilled();

    public boolean isPointsDrawn();

    public boolean isSpringsDrawn();

    public boolean isBoundsDrawn();

    public boolean isBodyInfoDrawn();
}

package model;

import java.util.ArrayList;

import model.geometry.Vector2D;
import model.softbody.ReadOnlySoftBody;

public interface ReadOnlyModel {
    ArrayList<ReadOnlySoftBody> getReadOnlySoftBodies();

    public int getId();

    public Vector2D getGravity();

    public boolean isBodyFilled();

    public boolean isPointsDrawn();

    public boolean isSpringsDrawn();

    public boolean isBoundsDrawn();

    public boolean isBodyInfoDrawn();
}

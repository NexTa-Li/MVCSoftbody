package model.masspoint;

import model.geometry.Point2D;
import model.geometry.Vector2D;

public interface ReadOnlyMassPoint {

    public boolean isCollided();

    public Point2D getPosition();

    public Double getPositionX();

    public Double getPositionY();

    public Vector2D getVelocity();

    public Double getVelocityX();

    public Double getVelocityY();

    public Vector2D getForce();

    public Double getForceX();

    public Double getForceY();
}

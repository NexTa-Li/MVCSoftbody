package model.masspoint;

import model.geometry.Point2D;
import model.geometry.Vector2D;

public interface ReadOnlyMassPoint {

    public boolean isCollided();

    public Point2D getPosition();

    public double getPositionX();

    public double getPositionY();

    public Vector2D getVelocity();

    public double getVelocityX();

    public double getVelocityY();

    public Vector2D getForce();

    public double getForceX();

    public double getForceY();
}

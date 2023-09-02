package model.softbody;

import java.util.ArrayList;

import model.masspoint.ReadOnlyMassPoint;

import java.awt.Graphics;
import java.awt.Rectangle;

public interface ReadOnlySoftBody {
    // Bare minimum for now
    public int getNumPoints();

    ArrayList<ReadOnlyMassPoint> getPoints();

    public Rectangle getBoundingBox();

    public void paintComponent(Graphics graphics);
}

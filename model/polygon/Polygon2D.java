package model.polygon;

import java.util.ArrayList;
import java.util.List;

import model.geometry.Point2D;
import model.geometry.Rectangle;
import model.masspoint.MassPoint;

public abstract class Polygon2D implements ReadOnlyPolygon2D {
    public int npoints;

    // coordinates of the points
    public List<MassPoint> points;

    protected Rectangle bounds; // double precision bounds
    private static final int MIN_LENGTH = 4;

    protected double mass;

    public Polygon2D(double mass) {
        this.points = new ArrayList<MassPoint>(MIN_LENGTH);
        this.bounds = new Rectangle();

        this.mass = mass;
    }

    /**
     * Translates the vertices of the {@code Polygon} by
     * {@code deltaX} along the x axis and by
     * {@code deltaY} along the y axis.
     * 
     * @param deltaX the amount to translate along the X axis
     * @param deltaY the amount to translate along the Y axis
     */
    public void translate(double deltaX, double deltaY) {
        for (int i = 0; i < npoints; i++) {
            points.get(i).getPosition().translate(deltaX, deltaY);
        }

        if (bounds != null) {
            bounds.translate(deltaX, deltaY);
        }
    }

    public Rectangle getBounds() {
        return bounds; // not safe, but faster
    }

    /*
     * Resizes the bounding box to accommodate the specified coordinates.
     * 
     * @param x,&nbsp;y the specified coordinates
     */
    void updateBounds(double x, double y) {
        if (x < bounds.x) {
            bounds.width = bounds.width + (bounds.x - x);
            bounds.x = x;
        } else {
            bounds.width = Math.max(bounds.width, x - bounds.x);
        }

        if (y < bounds.y) {
            bounds.height = bounds.height + (bounds.y - y);
            bounds.y = y;
        } else {
            bounds.height = Math.max(bounds.height, y - bounds.y);
        }
    }

    public void addPoint(double x, double y) {
        this.points.add(new MassPoint(new Point2D(x, y)));

        npoints++;
        if (bounds != null) {
            updateBounds(x, y);
        }
    }

    public boolean contains(double x, double y) {
        if (npoints <= 2 || !getBounds().contains(x, y)) {
            return false;
        }
        int hits = 0;

        double lastx = points.get(npoints - 1).getPosition().getX();
        double lasty = points.get(npoints - 1).getPosition().getY();
        double curx, cury;

        // Walk the edges of the polygon
        for (int i = 0; i < npoints; lastx = curx, lasty = cury, i++) {
            curx = points.get(i).getPosition().getX();
            cury = points.get(i).getPosition().getY();

            if (cury == lasty) {
                continue;
            }

            double leftx;
            if (curx < lastx) {
                if (x >= lastx) {
                    continue;
                }
                leftx = curx;
            } else {
                if (x >= curx) {
                    continue;
                }
                leftx = lastx;
            }

            double test1, test2;
            if (cury < lasty) {
                if (y < cury || y >= lasty) {
                    continue;
                }
                if (x < leftx) {
                    hits++;
                    continue;
                }
                test1 = x - curx;
                test2 = y - cury;
            } else {
                if (y < lasty || y >= cury) {
                    continue;
                }
                if (x < leftx) {
                    hits++;
                    continue;
                }
                test1 = x - lastx;
                test2 = y - lasty;
            }

            if (test1 < (test2 / (lasty - cury) * (lastx - curx))) {
                hits++;
            }
        }

        return ((hits & 1) != 0);
    }

    @Override
    public int[] getXArr() {
        int[] x = new int[npoints];
        for (int i = 0; i < npoints; i++) {
            x[i] = (int) points.get(i).getPosition().getX();
        }
        return x;
    }

    @Override
    public int[] getYArr() {
        int[] y = new int[npoints];
        for (int i = 0; i < npoints; i++) {
            y[i] = (int) points.get(i).getPosition().getY();
        }
        return y;
    }

    @Override
    public int getNumPoints() {
        return npoints;
    }
}

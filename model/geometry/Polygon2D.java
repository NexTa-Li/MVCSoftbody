package model.geometry;

import java.util.ArrayList;
import java.util.List;

public class Polygon2D implements ReadOnlyPolygon2D {
    public int npoints;

    // coordinates of the points
    public List<Point2D> points;

    protected Rectangle bounds; // double precision bounds
    private static final int MIN_LENGTH = 4;

    public Polygon2D() {
        this.points = new ArrayList<Point2D>(MIN_LENGTH);
        this.bounds = new Rectangle();
    }

    /**
     * Translates the vertices of the {@code Polygon} by
     * {@code deltaX} along the x axis and by
     * {@code deltaY} along the y axis.
     * 
     * @param deltaX the amount to translate along the X axis
     * @param deltaY the amount to translate along the Y axis
     */
    public void translate(int deltaX, int deltaY) {
        for (int i = 0; i < npoints; i++) {
            points.get(i).translate(deltaX, deltaY);
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
        this.points.add(new Point2D(x, y));

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

        double lastx = points.get(npoints - 1).x;
        double lasty = points.get(npoints - 1).y;
        double curx, cury;

        // Walk the edges of the polygon
        for (int i = 0; i < npoints; lastx = curx, lasty = cury, i++) {
            curx = points.get(i).x;
            cury = points.get(i).y;

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
            x[i] = (int) points.get(i).x;
        }
        return x;
    }

    @Override
    public int[] getYArr() {
        int[] y = new int[npoints];
        for (int i = 0; i < npoints; i++) {
            y[i] = (int) points.get(i).y;
        }
        return y;
    }

    @Override
    public int getNumPoints() {
        return npoints;
    }
}

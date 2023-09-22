package model.softbody;

import java.util.ArrayList;
import java.util.List;

import model.Spring;
import model.geometry.Point2D;
import model.masspoint.MassPoint;
import model.masspoint.ReadOnlyMassPoint;

public class SoftBody extends Body {
    final int NUM_POINTS;
    List<MassPoint> externalPoints;

    public SoftBody(double x, double y, int width, int height, double ks, double kd, int restLength, double mass) {
        super(x, y, mass, ks, kd);

        externalPoints = new ArrayList<>();
        createSoftBody(x, y, width, height, restLength);
        this.NUM_POINTS = points.size();
    }

    /**
     * Creates the points and springs of the softbody.
     * 
     * <p>
     * This method is called by the constructor.
     * The points are created in a circle around the center of the softbody, where
     * radius determines how far away the points are from the center.
     * 
     * <br>
     * The springs are created between each point and the next point in the list.
     * The index of each point is also added to the spring object, so that the
     * spring can access the points in the list.
     * </p>
     * 
     * @param x      x position of the center of the softbody
     * @param y      y position of the center of the softbody
     * @param radius radius of the softbody
     */
    void createSoftBody(double x, double y, int width, int height, int restLength) {
        int rows = height / restLength;
        int cols = width / restLength;

        double diagonalRestLength = Math.sqrt(2 * (restLength * restLength));

        int[][] pointIndices = new int[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                points.add(new MassPoint(x + j * restLength, y + i * restLength));
                pointIndices[i][j] = points.size() - 1;

                // springs to the right
                if (j < cols - 1) {
                    springs.add(new Spring((i * cols) + j, (i * cols) + j + 1, restLength));
                }

                // springs below
                if (i < rows - 1) {
                    springs.add(new Spring((i * cols) + j, ((i + 1) * cols) + j, restLength));
                }

                // springs diagonally below and to the right
                if (i < rows - 1 && j < cols - 1) {
                    springs.add(new Spring((i * cols) + j, ((i + 1) * cols) + j + 1, diagonalRestLength));
                }

                // springs diagonally below and to the left
                if (i < rows - 1 && j > 0) {
                    springs.add(new Spring((i * cols) + j, ((i + 1) * cols) + j - 1, diagonalRestLength));
                }
            }
        }

        List<Integer> pointIndex = new ArrayList<>();

        // Traverse the top edge
        for (int i = 0; i < cols; i++) {
            pointIndex.add(pointIndices[0][i]);
        }

        // Traverse the right edge
        for (int i = 1; i < rows; i++) {
            pointIndex.add(pointIndices[i][cols - 1]);
        }

        // Traverse the bottom edge
        if (rows > 1) { // Check if there is more than one row
            for (int i = cols - 2; i >= 0; i--) {
                pointIndex.add(pointIndices[rows - 1][i]);
            }
        }

        // Traverse the left edge
        if (cols > 1) { // Check if there is more than one column
            for (int i = rows - 2; i > 0; i--) {
                pointIndex.add(pointIndices[i][0]);
            }
        }

        for (Integer integer : pointIndex) {
            externalPoints.add(points.get(integer));
        }

        points.get(0).isFixed = true;
        points.get(cols - 1).isFixed = true;

    }

    @Override
    public ArrayList<MassPoint> points() {
        return new ArrayList<>(this.externalPoints);
    }

    @Override
    public ArrayList<ReadOnlyMassPoint> getPoints() {
        return new ArrayList<>(this.points);
    }

    public int[] getXArr() {
        int[] xArr = new int[externalPoints.size()];
        for (int i = 0; i < externalPoints.size(); i++) {
            double x = externalPoints.get(i).getPositionX();
            xArr[i] = (int) x;
        }
        return xArr;
    }

    public int[] getYArr() {
        int[] yArr = new int[externalPoints.size()];
        for (int i = 0; i < externalPoints.size(); i++) {
            double y = externalPoints.get(i).getPositionY();
            yArr[i] = (int) y;
        }
        return yArr;
    }
}
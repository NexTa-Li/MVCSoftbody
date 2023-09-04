package model.softbody;

import java.awt.geom.Line2D;

import model.geometry.Point2D;
import model.geometry.Vector2D;

public class SoftBodyUtil {
    private SoftBodyUtil() {
    }

    public static boolean checkCollision(Point2D position, SoftBody other) {
        // Check if the point is inside the bounding box of the other soft body
        if (!other.boundingBox.contains(position.getX(), position.getY())) {
            return false;
        }

        Point2D rayStart = new Point2D(0, position.getY());
        Point2D rayEnd = new Point2D(position);

        int points = other.points.size();
        int intersectionCount = 0;
        for (int i = 0; i < points; i++) {
            Point2D edgeStart = new Point2D(other.points.get(i).getPosition());
            Point2D edgeEnd = new Point2D(other.points.get((i + 1) % points).getPosition());

            if (SoftBodyUtil.checkIntersection(rayStart, rayEnd, edgeStart, edgeEnd)) {
                intersectionCount++;
            }
        }

        // If the number of intersections is odd, then the point is inside the soft body
        if (intersectionCount % 2 == 1) {
            return true;
        }

        // otherwise, the point is outside the soft body, so no collision
        return false;

    }

    public static boolean checkIntersection(Vector2D rayStart, Vector2D rayEnd, Vector2D edgeStart, Vector2D edgeEnd) {

        Line2D ray = new Line2D.Double(rayStart.getX(), rayStart.getY(), rayEnd.getX(), rayEnd.getY());
        Line2D edge = new Line2D.Double(edgeStart.getX(), edgeStart.getY(), edgeEnd.getX(), edgeEnd.getY());

        return ray.intersectsLine(edge);
    }

    // TODO: Fix this method to work when the horizontal ray is at the same y
    // coordinate as the invading point
    public static boolean checkIntersection(Point2D rayStart, Point2D rayEnd, Point2D edgeStart, Point2D edgeEnd) {

        Line2D ray = new Line2D.Double(rayStart.getX(), rayStart.getY(), rayEnd.getX(), rayEnd.getY());
        Line2D edge = new Line2D.Double(edgeStart.getX(), edgeStart.getY(), edgeEnd.getX(), edgeEnd.getY());

        return ray.intersectsLine(edge);
    }

    public static Point2D closestPointOnLineSegment(Point2D p, Point2D a, Point2D b) {
        double x0 = p.getX();
        double y0 = p.getY();
        double lx1 = a.getX();
        double ly1 = a.getY();
        double lx2 = b.getX();
        double ly2 = b.getY();

        double dx = lx2 - lx1;
        double dy = ly2 - ly1;

        // Calculate the squared length of the line segment
        double segmentLengthSquared = dx * dx + dy * dy;

        // Handle degenerate cases where the segment has zero length
        if (segmentLengthSquared == 0) {
            return new Point2D(lx1, ly1);
        }

        // Calculate the vector from the start of the segment to the point
        double vectorX = x0 - lx1;
        double vectorY = y0 - ly1;

        // Calculate the dot product between the vector and the direction
        double dotProduct = vectorX * dx + vectorY * dy;

        // Calculate the closest point on the line segment
        double t = dotProduct / segmentLengthSquared;
        t = Math.max(0, Math.min(1, t)); // Clamp t to the range [0, 1]
        double closestX = lx1 + t * dx;
        double closestY = ly1 + t * dy;

        return new Point2D(closestX, closestY);
    }

    /**
     * checks if the centre of 2 softbodies are within 5 units of each other and
     * returns true if they are, false otherwise.
     * 
     * <p>
     * if the centre of both bounding boxes are within 5 units of each other they
     * are merged, where 5 units is the margin of error, since no 2 bodies bounding
     * boxes centres will ever be within 5 units of each other
     * </p>
     * 
     * @param body  the first softbody
     * @param other the second softbody
     * @return true if the softbodies are merged, false otherwise
     */
    public static boolean isMerged(SoftBody body, SoftBody other) {
        if (body == other) {
            return true;
        }

        Point2D bodyCentre = new Point2D(body.getBoundingBox().getCenterX(), body.getBoundingBox().getCenterY());
        Point2D otherCentre = new Point2D(other.getBoundingBox().getCenterX(), other.getBoundingBox().getCenterY());

        if (bodyCentre.distance(otherCentre) > 5) {
            return false;
        }

        return true;
    }
}

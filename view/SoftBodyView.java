package view;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JPanel;

import controller.SoftBodyController;
import model.ModelConfig;
import model.ReadOnlyModel;
import model.Spring;
import model.masspoint.ReadOnlyMassPoint;
import model.polygon.ReadOnlyPolygon2D;
import model.softbody.ReadOnlySoftBody;

public class SoftBodyView extends JPanel implements Runnable, ViewConfig {
    ReadOnlyModel model;
    SoftBodyController controller;
    List<ReadOnlySoftBody> softBodies;
    List<ReadOnlyPolygon2D> polygons;

    Thread thread;

    public SoftBodyView(ReadOnlyModel model, SoftBodyController controller) {
        this.setPreferredSize(new Dimension(PANEL_WIDTH, PANEL_HEIGHT));
        this.model = model;
        this.softBodies = model.getReadOnlySoftBodies();
        this.polygons = model.getReadOnlyPolygons();
        this.controller = controller;
        this.setBackground(Color.black);
        this.setFocusable(true);
        this.requestFocus();

        this.addKeyListener(controller);
        this.addMouseListener(controller);
        this.addMouseMotionListener(controller);
        this.addMouseWheelListener(controller);
        this.thread = new Thread(this);
        this.thread.start();

        JFrame frame = new JFrame("Soft Body");
        frame.add(this);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }

    @Override
    public void run() {
        double drawInterval = 1000000000 / REFRESH_RATE;
        double delta = 0;
        long lastTime = System.nanoTime();
        long currentTime;

        while (thread != null) {
            currentTime = System.nanoTime();

            delta += (currentTime - lastTime) / drawInterval;

            lastTime = currentTime;

            if (delta >= 1) {
                revalidate();
                repaint();
                delta--;
            }
        }
    }

    @Override
    protected void paintComponent(Graphics graphics) {
        super.paintComponent(graphics);
        // graphics 2d
        Graphics2D g = (Graphics2D) graphics;
        g.setStroke(new BasicStroke(2));
        List<ReadOnlyMassPoint> points;
        List<Spring> springs;
        ReadOnlySoftBody body;

        // draw polygons
        for (int i = 0; i < polygons.size(); i++) {
            g.setColor(Color.WHITE);

            g.fillPolygon(polygons.get(i).getXArr(), polygons.get(i).getYArr(), polygons.get(i).getNumPoints());
        }

        for (int i = 0; i < softBodies.size(); i++) {
            body = softBodies.get(i);
            points = body.getPoints();
            springs = body.getSprings();

            if (model.isBodyFilled()) {
                g.setColor(Color.gray);
                g.fillPolygon(body.getXArr(), body.getYArr(), body.getYArr().length);
            }

            if (model.isSpringsDrawn()) {
                g.setColor(Color.WHITE);

                for (int j = 0; j < springs.size(); j++) {
                    g.drawLine((int) points.get(springs.get(j).getP1()).getPosition().getX(),
                            (int) points.get(springs.get(j).getP1()).getPosition().getY(),
                            (int) points.get(springs.get((j)).getP2()).getPosition().getX(),
                            (int) points.get(springs.get((j)).getP2()).getPosition().getY());
                }
            }

            if (model.isPointsDrawn()) {
                g.setColor(Color.red);

                for (int j = 0; j < points.size(); j++) {

                    g.fillOval((int) (points.get(j).getPositionX() - ModelConfig.POINT_SIZE),
                            (int) (points.get(j).getPositionY() - ModelConfig.POINT_SIZE), (int) (2 *
                                    ModelConfig.POINT_SIZE),
                            (int) (2 * ModelConfig.POINT_SIZE));
                }

            }

            // draw bounding box
            if (model.isBoundsDrawn()) {
                g.setColor(Color.green);
                g.drawRect(body.getBoundingBox().xToInt(), body.getBoundingBox().yToInt(),
                        body.getBoundingBox().widthToInt(), body.getBoundingBox().heightToInt());
            }
        }

        if (model.isBodyInfoDrawn()) {
            renderSoftBodyInfo(g);
        }

    }

    public void renderSoftBodyInfo(Graphics2D g) {
        g.setColor(new Color(255, 255, 255, 122));
        g.fillRect(0, PANEL_HEIGHT / 2, 235, PANEL_HEIGHT / 2);

        g.setColor(Color.black);

        g.drawString("Selected Body: ", 10, PANEL_HEIGHT / 2 + 20);
        g.drawString("" + model.getId(), 120, PANEL_HEIGHT / 2 + 20);

        ReadOnlySoftBody body = softBodies.get(model.getId());

        g.drawString("Pressure: ", 10, PANEL_HEIGHT / 2 + 40);
        g.drawString(String.format("%,.2f", body.getPressure()) + "", 120, PANEL_HEIGHT / 2 + 40);

        g.drawString("Mass: ", 10, PANEL_HEIGHT / 2 + 60);
        g.drawString(String.format("%,.2f", body.getMass()) + "", 120, PANEL_HEIGHT / 2 + 60);

        g.drawString("Mass Points: ", 10, PANEL_HEIGHT / 2 + 80);
        g.drawString(body.getPoints().size() + "", 120, PANEL_HEIGHT / 2 + 80);

        g.drawString("Spring Constant: ", 10, PANEL_HEIGHT / 2 + 100);
        g.drawString(String.format("%,.2f", body.getSpringConstant()) + "", 120, PANEL_HEIGHT / 2 + 100);

        g.drawString("Spring Damping: ", 10, PANEL_HEIGHT / 2 + 120);
        g.drawString(String.format("%,.2f", body.getSpringDamping()) + "", 120, PANEL_HEIGHT / 2 + 120);

        g.drawString("Spring Rest Length: ", 10, PANEL_HEIGHT / 2 + 140);
        g.drawString(String.format("%,.5f", body.getSpringRestLength()) + "", 120, PANEL_HEIGHT / 2 + 140);

        // String colorString =
        // softBodies.get(selectedSoftbodyIndex).averageColor.toString();
        // g.drawString("Color: ", 10, PANEL_HEIGHT / 2 + 160);
        // g.drawString(colorString.substring(colorString.indexOf("[")), 120,
        // PANEL_HEIGHT / 2 + 160);
        // g.setColor(softBodies.get(selectedSoftbodyIndex).averageColor);
        // g.fillRect(10, PANEL_HEIGHT / 2 + 165, 215, 20);

        // drawing selected point
        g.setColor(Color.black);
        g.drawString("Selected Point: ", 10, PANEL_HEIGHT / 2 + 200);
        g.drawString("" + model.getSelectedPoint(), 120, PANEL_HEIGHT / 2 + 200);

        // // draw a circle around the selected point
        g.setColor(Color.black);
        g.drawOval(
                (int) (softBodies.get(model.getId()).getPoints().get(model.getSelectedPoint()).getPosition().getX()
                        - 8),
                (int) (softBodies.get(model.getId()).getPoints().get(model.getSelectedPoint()).getPosition().getY()
                        - 8),
                16, 16);

        // g.drawString("Velocity: ", 10, PANEL_HEIGHT / 2 + 220);
        // g.drawString(df.format(softBodies.get(selectedSoftbodyIndex).points.get(selectedPoint).velocity.x)
        // + ", "
        // +
        // df.format(softBodies.get(selectedSoftbodyIndex).points.get(selectedPoint).velocity.y),
        // 120,
        // PANEL_HEIGHT / 2 + 220);

        // g.drawString("Force: ", 10, PANEL_HEIGHT / 2 + 240);
        // g.drawString(df.format(softBodies.get(selectedSoftbodyIndex).points.get(selectedPoint).force.x)
        // + ", "
        // +
        // df.format(softBodies.get(selectedSoftbodyIndex).points.get(selectedPoint).force.y),
        // 120,
        // PANEL_HEIGHT / 2 + 240);

        // g.drawString("Collided: ", 10, PANEL_HEIGHT / 2 + 260);
        // g.drawString("" +
        // softBodies.get(selectedSoftbodyIndex).points.get(selectedPoint).collided,
        // 120,
        // PANEL_HEIGHT / 2 + 260);

        // String pointColor =
        // softBodies.get(selectedSoftbodyIndex).points.get(selectedPoint).color.toString();
        // g.drawString("Point Color: ", 10, PANEL_HEIGHT / 2 + 280);
        // g.drawString(pointColor.substring(pointColor.indexOf("[")), 120, PANEL_HEIGHT
        // / 2 + 280);
        // g.setColor(softBodies.get(selectedSoftbodyIndex).points.get(selectedPoint).color);
        // g.fillRect(10, PANEL_HEIGHT / 2 + 285, 215, 20);

        // // draw desired volume and volume
        // g.setColor(Color.black);
        // g.drawString("Desired Volume: ", 10, PANEL_HEIGHT / 2 + 320);
        // g.drawString(df.format(softBodies.get(selectedSoftbodyIndex).DESIRED_VOLUME)
        // + "", 120, PANEL_HEIGHT / 2 + 320);

        // g.drawString("Volume: ", 10, PANEL_HEIGHT / 2 + 340);
        // g.drawString(df.format(softBodies.get(selectedSoftbodyIndex).volume) + "",
        // 120, PANEL_HEIGHT / 2 + 340);

        // // Draw velocity vector as a line from the point

        // int x1 = (int)
        // Math.round(softBodies.get(selectedSoftbodyIndex).points.get(selectedPoint).getPositionX());
        // int y1 = (int)
        // Math.round(softBodies.get(selectedSoftbodyIndex).points.get(selectedPoint).getPositionY());
        // int x2 = x1
        // + (int)
        // Math.round(softBodies.get(selectedSoftbodyIndex).points.get(selectedPoint).getVelocityX());
        // int y2 = y1
        // + (int)
        // Math.round(softBodies.get(selectedSoftbodyIndex).points.get(selectedPoint).getVelocityY());

        // g.setColor(Color.blue);
        // g.drawLine(x1, y1, x2, y2);

        // // Draw average velocity vector as a line from the point

        // x1 = (int)
        // Math.round(softBodies.get(selectedSoftbodyIndex).boundingBox.getCenterX());
        // y1 = (int)
        // Math.round(softBodies.get(selectedSoftbodyIndex).boundingBox.getCenterY());
        // x2 = x1 + (int)
        // Math.round(softBodies.get(selectedSoftbodyIndex).averageVelocity.getX());
        // y2 = y1 + (int)
        // Math.round(softBodies.get(selectedSoftbodyIndex).averageVelocity.getY());

        // g.setColor(Color.green);
        // g.drawLine(x1, y1, x2, y2);

        // x1 = (int)
        // Math.round(softBodies.get(selectedSoftbodyIndex).points.get(selectedPoint).getPositionX());
        // y1 = (int)
        // Math.round(softBodies.get(selectedSoftbodyIndex).points.get(selectedPoint).getPositionY());
        // x2 = x1 + (int)
        // Math.round(softBodies.get(selectedSoftbodyIndex).points.get(selectedPoint).getForceX());
        // y2 = y1 + (int)
        // Math.round(softBodies.get(selectedSoftbodyIndex).points.get(selectedPoint).getForceY());

        // g.setColor(Color.red);
        // g.drawLine(x1, y1, x2, y2);
    }
}
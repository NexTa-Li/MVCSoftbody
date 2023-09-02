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
import model.ReadOnlyModel;
import model.softbody.ReadOnlySoftBody;

public class SoftBodyView extends JPanel implements Runnable, ViewConfig {
    ReadOnlyModel model;
    SoftBodyController controller;
    List<ReadOnlySoftBody> softBodies;

    Thread thread;

    public SoftBodyView(ReadOnlyModel model, SoftBodyController controller) {
        this.setPreferredSize(new Dimension(PANEL_WIDTH, PANEL_HEIGHT));
        this.model = model;
        this.softBodies = model.getReadOnlySoftBodies();
        this.setBackground(Color.black);
        this.setFocusable(true);
        this.requestFocus();

        this.addKeyListener(controller);
        this.addMouseListener(controller);
        this.addMouseMotionListener(controller);

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

        for (int i = 0; i < softBodies.size(); i++) {
            softBodies.get(i).paintComponent(g);
        }
    }

}
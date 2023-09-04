package model;

import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.ArrayList;
import java.util.HashSet;

import model.geometry.Point2D;
import model.geometry.Vector2D;
import model.softbody.ReadOnlySoftBody;
import model.softbody.SoftBody;
import view.ViewConfig;

public class SoftBodyModel implements ReadOnlyModel, Runnable, ModelConfig {

    public static Vector2D gravity = new Vector2D(0, 60);
    List<SoftBody> softBodies;
    Thread thread;

    // graphical settings that can be changed by the controller
    public static boolean fillSofbtody = true;
    public static boolean drawPoints = false;
    public static boolean drawSprings = true;
    public static boolean drawBoundingBox = false;

    public SoftBodyModel() {

        createSoftBodies();
        thread = new Thread(this);

        if (!DEBUG_MODE) {
            thread.start();
        }
    }

    void createSoftBodies() {
        this.softBodies = new ArrayList<SoftBody>();

        if (!RANDOM_POSITIONS) {

            int X = 150;
            int Y = ViewConfig.PANEL_HEIGHT / 2;

            for (int i = 0; i < NUM_SOFTBODIES; i++) {

                SoftBody softBody = new SoftBody(X, Y, NUM_POINTS, RADIUS, MASS, SPRING_CONSTANT,
                        SPRING_DAMPING, FINAL_PRESSURE, softBodies);

                // set random color
                X += 100;
                Y += 13;
                softBodies.add(softBody);
            }
            return;
        }

        // Random positions, but with a seed
        Random random = new Random();
        random.setSeed(3);

        for (int i = 0; i < NUM_SOFTBODIES; i++) {

            int rX = (random.nextInt(ViewConfig.PANEL_WIDTH));
            int rY = (random.nextInt(ViewConfig.PANEL_HEIGHT));

            SoftBody softBody = new SoftBody(rX, rY, NUM_POINTS, RADIUS, MASS, SPRING_CONSTANT,
                    SPRING_DAMPING, FINAL_PRESSURE, softBodies);

            softBodies.add(softBody);
        }
    }

    public void idle() {
        for (int i = 0; i < softBodies.size(); i++) {
            softBodies.get(i).idle();
        }

        for (int i = softBodies.size() - 1; i >= 0; i--) {
            // softBodies.get(i).idle();
        }

        // if (count % 800 == 0) {
        // gravity.multiplyY(-1);
        // count = 0;
        // }
        // count++;
    }

    @Override
    public void run() {
        double drawInterval = 1000000000 / TICKRATE; // FPS = 240
        double delta = 0;
        long lastTime = System.nanoTime();
        long currentTime;

        while (thread != null) {
            currentTime = System.nanoTime();

            delta += (currentTime - lastTime) / drawInterval;

            lastTime = currentTime;

            // Update the game logic when enough time has passed for a frame
            if (delta >= 1.0) {
                idle(); // Update game logic and rendering
                delta--;
            }

            // Calculate the time to sleep in order to achieve the desired frame rate
            long sleepTime = (long) (drawInterval - (System.nanoTime() - currentTime));
            if (sleepTime > 0) {
                try {
                    Thread.sleep(sleepTime / 1000000); // Convert nanoseconds to milliseconds
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    public List<SoftBody> getSoftBodies() {
        return this.softBodies;
    }

    @Override
    public ArrayList<ReadOnlySoftBody> getReadOnlySoftBodies() {
        return new ArrayList<ReadOnlySoftBody>(softBodies);
    }

    @Override
    public int getId() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getId'");
    }

    @Override
    public double getPressure() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPressure'");
    }

    @Override
    public double getNumPoints() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getNumPoints'");
    }

    @Override
    public double getSpringConstant() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSpringConstant'");
    }

    @Override
    public double getSpringDamping() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSpringDamping'");
    }

    @Override
    public double getMass() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMass'");
    }

    @Override
    public double getRadius() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRadius'");
    }

}

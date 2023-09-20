package model;

import java.util.List;
import java.util.Random;
import java.util.ArrayList;

import model.geometry.Vector2D;
import model.polygon.BouncePad;
import model.polygon.Polygon2D;
import model.polygon.ReadOnlyPolygon2D;
import model.softbody.ReadOnlySoftBody;
import model.softbody.SoftBody;
import model.softbody.SoftBodyHandler;
import model.softbody.SoftBodyUtil;
import view.ViewConfig;

public class SoftBodyModel implements ReadOnlyModel, Runnable, ModelConfig {

    public static Vector2D gravity = new Vector2D(0, 250);
    public List<SoftBody> softBodies;
    public List<SoftBodyHandler> softBodyHandlers;
    public List<Polygon2D> polygons;
    Thread thread;

    public BouncePad p1 = new BouncePad(2.0, new Vector2D(0, -5), 200, 5);

    // graphical settings that can be changed by the controller
    public boolean bodyFilled = true;
    public boolean pointsDrawn = false;
    public boolean springsDrawn = false;
    public boolean boundsDrawn = false;
    public boolean drawBodyInfo = false;

    public int selectedSoftbodyIndex = 0;

    public SoftBodyModel() {

        createPolygons();

        createSoftBodies();

        // softBodies.add(new SoftBody(500, 200, 64, RADIUS, MASS, SPRING_CONSTANT,
        // 60, FINAL_PRESSURE, softBodies, polygons));

        thread = new Thread(this);

        if (!DEBUG_MODE) {
            thread.start();
        }
    }

    void createPolygons() {
        this.polygons = new ArrayList<Polygon2D>();

        p1.addPoint(300, 500);

        p1.addPoint(500, 400);
        p1.addPoint(700, 500);
        p1.addPoint(500, 600);

        polygons.add(p1);
    }

    void createSoftBodies() {
        this.softBodies = new ArrayList<SoftBody>();
        this.softBodyHandlers = new ArrayList<SoftBodyHandler>();

        if (!RANDOM_POSITIONS) {

            int X = 150;
            int Y = ViewConfig.PANEL_HEIGHT / 2;

            for (int i = 0; i < NUM_SOFTBODIES; i++) {

                SoftBody softBody = new SoftBody(X, Y, NUM_POINTS, RADIUS, MASS, SPRING_CONSTANT,
                        SPRING_DAMPING, FINAL_PRESSURE);

                // set random color
                X += 300;
                // Y += 13;
                softBodies.add(softBody);
                softBodyHandlers.add(new SoftBodyHandler(softBody, this));
            }
            return;
        }

        // Random positions, but with a seed
        Random random = new Random();
        random.setSeed(3);

        int i = 0;
        int numBodies = NUM_SOFTBODIES;
        while (i < numBodies) {

            int rX = (random.nextInt(ViewConfig.PANEL_WIDTH));
            int rY = (random.nextInt(ViewConfig.PANEL_HEIGHT));

            for (int j = 0; j < i; j++) {
                if (i - 1 == j)
                    continue;

                if (SoftBodyUtil.isMerged(softBodies.get(i - 1), softBodies.get(j))) {
                    System.out.println("Softbody " + i + " and " + j + " are merged");
                    numBodies++;
                    break;
                }
            }
            SoftBody softBody = new SoftBody(rX, rY, NUM_POINTS, RADIUS, MASS, SPRING_CONSTANT,
                    SPRING_DAMPING, FINAL_PRESSURE);

            softBodies.add(softBody);
            softBodyHandlers.add(new SoftBodyHandler(softBody, this));
            i++;
        }
    }

    public void idle() {
        for (int i = 0; i < softBodyHandlers.size(); i++) {
            softBodyHandlers.get(i).idle();
        }

        p1.idle();
    }

    @Override
    public void run() {
        double drawInterval = 1000000000 / TICKRATE; // FPS = 240
        double delta = 0;
        long lastTime = System.nanoTime();
        long currentTime;

        long time = 0;
        long totalTime = 0;

        int benchmarkInterval = 1000;
        int count = 1;

        while (thread != null) {
            currentTime = System.nanoTime();

            delta += (currentTime - lastTime) / drawInterval;

            lastTime = currentTime;

            // Update the game logic when enough time has passed for a frame
            if (delta >= 1.0) {
                long begin = System.nanoTime(); // Testing how long the idle method takes
                idle(); // Update game logic and rendering
                delta--;

                long end = System.nanoTime();
                time += end - begin;
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

            if (count % benchmarkInterval == 0) {
                System.out.println("Idle method took " + time / 1e6 + " ms");
                totalTime += time;
                System.out.println("Average time: " + ((totalTime) / 1e6) / count + " ms");
                time = 0;
            }

            count++;
        }
    }

    public List<SoftBody> getSoftBodies() {
        return this.softBodies;
    }

    public List<SoftBodyHandler> getSoftBodyHandlers() {
        return this.softBodyHandlers;
    }

    public List<Polygon2D> getPolygons() {
        return this.polygons;
    }

    @Override
    public ArrayList<ReadOnlySoftBody> getReadOnlySoftBodies() {
        return new ArrayList<ReadOnlySoftBody>(softBodies);
    }

    @Override
    public int getId() {
        return this.selectedSoftbodyIndex;
    }

    @Override
    public Vector2D getGravity() {
        return new Vector2D(SoftBodyModel.gravity);
    }

    @Override
    public boolean isBodyFilled() {
        return this.bodyFilled;
    }

    @Override
    public boolean isPointsDrawn() {
        return this.pointsDrawn;
    }

    @Override
    public boolean isSpringsDrawn() {
        return this.springsDrawn;
    }

    @Override
    public boolean isBoundsDrawn() {
        return this.boundsDrawn;
    }

    @Override
    public boolean isBodyInfoDrawn() {
        return this.drawBodyInfo;
    }

    @Override
    public ArrayList<ReadOnlyPolygon2D> getReadOnlyPolygons() {
        return new ArrayList<ReadOnlyPolygon2D>(polygons);
    }

}

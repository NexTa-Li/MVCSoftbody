package model;

public interface ModelConfig {
    // Physical constants
    static final int FILL_DURATION = 50; // the higher, the better (helps with volume calculations)
    static final double USER_FORCE = 250;
    static final double BOUNCINESS = 0.9; // 0.90 is a good value
    static final double SURFACE_FRICTION_COEFFICIENT = 0.02; // 0.01 is a good value
    static final int NUM_SOFTBODIES = 2;

    static final int NUM_POINTS = 20;
    static final double RADIUS = 45.0; // controls spacing between points during the first tick
    static final double MASS = 0.5; // collisions are more reliable with smaller mass
    static final double SPRING_CONSTANT = 700.0;
    static final double SPRING_DAMPING = 30.0;
    static final double FINAL_PRESSURE = 100_000.0;

    static final int WALL_THICKNESS = 3;

    static final int FPS = 240;
    static final double timestep = 0.005; // 0.005 is a decent value

    /**
     * The radius of each mass point in the soft body.
     * This is used to resolve collisions between two mass points
     */
    static final double MASS_POINT_RADIUS = 0.1; // How big each mass point is
    static final float POINT_SIZE = 2.0f;

    static final boolean DEBUG_MODE = true;
    static final boolean RANDOM_POSITIONS = false;
}

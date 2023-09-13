package model;

public interface ModelConfig {
    // Physical constants
    static final int FILL_DURATION = 200; // the higher, the better (helps with volume calculations)
    static final double USER_FORCE = 250;
    public static final double BOUNCINESS = 0.97; // 0.95 is a good value
    public static final double SURFACE_FRICTION_COEFFICIENT = 0.02; // 0.01 is a good value
    public static final double AIR_FRICTION_COEFFICIENT = 0.0001; // 0.0001 is a good value
    static final int NUM_SOFTBODIES = 4;

    static final int NUM_POINTS = 48;

    /**
     * The Radius the softbody will try to maintain. This radius will not always be
     * achieved
     * since the pressure forces may be acting against the spring forces.
     * 
     * This determines the radius of the softbody in the first tick.
     * This means that the resting length of each spring is also
     * determined by this value.
     */
    static final double RADIUS = 45.0;

    /**
     * The Mass of each mass point in the softbody. This determines how much the
     * mass points will be affected by the forces acting on them.
     * 
     * A higher mass means that the mass point will be affected more by the forces.
     * 
     * As of 2023-09-05, a mass of 0.5 is a good value, since the smaller mass
     * values result in better collisions.
     * 
     * New findings show that a larger mass will allow a higher damping value.
     */
    static final double MASS = 3.0; // collisions are more reliable with smaller mass

    /**
     * The spring constant of the softbody. this determines how much the springs
     * will try to maintain their resting length.
     */
    static final double SPRING_CONSTANT = 250_000.0;
    static final double SPRING_DAMPING = 30.0;
    static final double FINAL_PRESSURE = 50_000_000.0;

    static final int WALL_THICKNESS = 3;

    static final int TICKRATE = 512;
    static final double timestep = 0.002; // 0.005 is a decent value

    /**
     * The radius of each mass point in the soft body.
     * This is used to resolve collisions between two mass points
     */
    static final double MASS_POINT_RADIUS = 0.1;
    static final float POINT_SIZE = 4.0f; // The visual size of each mass point

    /**
     * Debugging option, if true, the simulations thread will not be started.
     * instead you have to go through each tick manually by pressing the space bar
     */
    static final boolean DEBUG_MODE = false;

    /**
     * Determines if the positions are random or not, when they aren't random the
     * bodies will spawn in a straight line from left to right. This is useful for
     * debugging collisions.
     */
    static final boolean RANDOM_POSITIONS = false;
}

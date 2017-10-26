package es.usc.citius.lab.motionplanner.core.spatial;

public interface State extends Pose{

    /**
     * @return reflected State respect to the XZ plane
     */
    public State symmetricPlaneXZ();

    /**
     * @return reflected State respect to the YZ plane
     */
    public State symmetricPlaneYZ();

    /**
     * @return reflected State, respect to the XY plane
     */
    public State symmetricPlaneXY();

    /**
     * Obtains the state adding the X, Y, Z values of the state and the given point.
     *
     * @param move coordinates to add (3D)
     * @return state resulting of the addition operation
     */
    @Override
    public State add(Point move);

    /**
     * Obtains the state subtracting the X, Y, Z values of the state and the given point.
     *
     * @param move coordinates to subtract (3D)
     * @return state resulting of the subtraction operation
     */
    @Override
    public State subtract(Point move);

    /**
     * Rotates the position and heading of the state keeping the velocities.
     *
     * @param yaw rotation in yaw
     * @param roll rotation in roll
     * @param pitch rotation in pitch
     * @return rotated state (position and heading) keeping the velocities as they are in local frame
     */
    public State rotate(float yaw, float pitch, float roll);

    /**
     * Returns the symmetry plane of a State object. It can be symmetric respect to the XZ plane or the YZ plane (depends on its proximity to each one).
     *
     * @return 0 if symmetry axis is X, 1 if it is Y
     */
    public int symmetryPlane();

    /**
     * Creates a copy of this element.
     *
     * @return copy of this instance.
     */
    public State clone();

    /**
     * Mandatory getters
     */
    public float getVx();

    public float getVy();

    public float getVz();

    public float getVroll();

    public float getVpitch();

    public float getVyaw();

}

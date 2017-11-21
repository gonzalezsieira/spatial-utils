package es.usc.citius.lab.motionplanner.core.spatial;

public interface Pose extends Point{

    public float getYaw();

    public float getPitch();

    public float getRoll();

    /**
     * Obtains the state adding the X, Y, Z values of the state and the given point.
     *
     * @param move coordinates to add (3D)
     * @return pose resulting of the addition operation
     */
    @Override
    public Pose add(Point move);

    /**
     * Obtains the state subtracting the X, Y, Z values of the state and the given point.
     *
     * @param move coordinates to subtract (3D)
     * @return pose resulting of the subtraction operation
     */
    @Override
    public Pose subtract(Point move);

    /**
     * Rotates the position and heading of the pose.
     *
     * @param yaw rotation in yaw
     * @param roll rotation in roll
     * @param pitch rotation in pitch
     * @return rotated pose (position and heading)
     */
    @Override
    public Pose rotate(float yaw, float pitch, float roll);

}

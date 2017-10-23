package es.usc.citius.lab.motionplanner.core.spatial;

public interface Geometry<S> {

    /**
     * @return reflected Geometry respect to the XZ plane
     */
    public S symmetricPlaneXZ();

    /**
     * @return reflected Geometry respect to the YZ plane
     */
    public S symmetricPlaneYZ();

    /**
     * @return reflected Geometry, respect to the XY plane
     */
    public S symmetricPlaneXY();

    /**
     * Obtains the state adding the X, Y, Z values of the state and the given point.
     *
     * @param move coordinates to add (3D)
     * @return state resulting of the addition operation
     */
    public S add(Point3D move);

    /**
     * Obtains the state adding the X, Y, Z values of the state and the given point. Does not create
     * a new instance, it modifies the given one instead.
     *
     * @param move coordinates to add (3D)
     */
    public void staticAdd(Point3D move);

    /**
     * Obtains the state subtracting the X, Y, Z values of the state and the given point.
     *
     * @param move coordinates to subtract (3D)
     * @return state resulting of the subtraction operation
     */
    public S subtract(Point3D move);

    /**
     * Obtains the state subtracting the X, Y, Z values of the state and the given point. Does not create
     * a new instance, it modifies the given one instead.
     *
     * @param move coordinates to subtract (3D)
     */
    public void staticSubtract(Point3D move);

    /**
     * Rotates the position and heading of the state keeping the velocities.
     *
     * @param yaw rotation in yaw
     * @param roll rotation in roll
     * @param pitch rotation in pitch
     * @return rotated state (position and heading) keeping the velocities as they are in local frame
     */
    public S rotate(float yaw, float pitch, float roll);

    /**
     * Returns the symmetry plane of a Geometry object. It can be symmetric respect to the XZ plane or the YZ plane (depends on its proximity to each one).
     *
     * @return 0 if symmetry axis is X, 1 if it is Y
     */
    public int symmetryPlane();

    /**
     * Creates a copy of this element.
     *
     * @return copy of this instance.
     */
    public S clone();

}

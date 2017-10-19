package es.usc.citius.lab.motionplanner.core.spatial;

public interface Geometry<S extends Object> extends Cloneable {

    /**
     * @return reflected Geometry respect to the XZ plane
     */
    public S symmetricAxisXZ();

    /**
     * @return reflected Geometry respect to the YZ plane
     */
    public S symmetricAxisYZ();

    /**
     * @return reflected Geometry, respect to the XY plane
     */
    public S symmetricAxisXY();

    /**
     * Obtains the state adding the X, Y, Z values of the state and the given point.
     *
     * @param move coordinates to add (3D)
     * @return state resulting of the addition operation
     */
    public S add(Point3D move);

    /**
     * Obtains the state adding the X, Y values of the state and the given point.
     *
     * @param move coordinates to add (2D)
     * @return state resulting of the addition operation
     */
    public S add(Point2D move);

    /**
     * Obtains the state adding the X, Y, Z values of the state and the given point. Does not create
     * a new instance, it modifies the given one instead.
     *
     * @param move coordinates to add (3D)
     */
    public void staticAdd(Point3D move);

    /**
     * Obtains the state adding the X, Y values of the state and the given point. Does not create
     * a new instance, it modifies the given one instead.
     *
     * @param move coordinates to add (2D)
     */
    public void staticAdd(Point2D move);

    /**
     * Obtains the state subtracting the X, Y, Z values of the state and the given point.
     *
     * @param move coordinates to subtract (3D)
     * @return state resulting of the subtraction operation
     */
    public S subtract(Point3D move);

    /**
     * Obtains the state subtracting the X, Y values of the state and the given point.
     *
     * @param move coordinates to subtract (2D)
     * @return state resulting of the subtraction operation
     */
    public S subtract(Point2D move);

    /**
     * Obtains the state subtracting the X, Y, Z values of the state and the given point. Does not create
     * a new instance, it modifies the given one instead.
     *
     * @param move coordinates to subtract (3D)
     */
    public void staticSubtract(Point3D move);

    /**
     * Obtains the state subtracting the X, Y values of the state and the given point. Does not create
     * a new instance, it modifies the given one instead.
     *
     * @param move coordinates to subtract (2D)
     */
    public void staticSubtract(Point2D move);

    /**
     * Rotates the position and heading of the state keeping the velocities.
     *
     * @param yaw rotation in yaw
     * @param roll rotation in roll
     * @param pitch rotation in pitch
     * @return rotated state (position and heading) keeping the velocities as they are in local frame
     */
    public S rotate(float yaw, float pitch, float roll);

}

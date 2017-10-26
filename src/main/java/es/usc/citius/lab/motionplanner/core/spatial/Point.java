package es.usc.citius.lab.motionplanner.core.spatial;

public interface Point {

    public float getX();

    public float getY();

    public float getZ();

    public float distance(Point other);

    /**
     * Obtains the state adding the X, Y, Z values of the state and the given point.
     *
     * @param move coordinates to add (3D)
     * @return point resulting of the addition operation
     */
    public Point add(Point move);

    /**
     * Obtains the state adding the X, Y, Z values of the state and the given point. Does not create
     * a new instance, it modifies the given one instead.
     *
     * @param move coordinates to add (3D)
     */
    public void staticAdd(Point move);

    /**
     * Obtains the state subtracting the X, Y, Z values of the state and the given point.
     *
     * @param move coordinates to subtract (3D)
     * @return point resulting of the subtraction operation
     */
    public Point subtract(Point move);

    /**
     * Obtains the state subtracting the X, Y, Z values of the state and the given point. Does not create
     * a new instance, it modifies the given one instead.
     *
     * @param move coordinates to subtract (3D)
     */
    public void staticSubtract(Point move);

}

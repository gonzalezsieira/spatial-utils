package es.usc.citius.lab.motionplanner.core.spatial;

/**
 * Definition of the basic operations that must be implemented in a Vector subclass.
 */
public interface Vector extends Rotable<Vector>{

    /**
     * Performs the dot product between two instances of Vector.
     *
     * @param other other instance of {@link Vector}
     * @return
     */
    public float dotProduct(Vector other);

    /**
     * Performs the dot product between two Vector and Point.
     *
     * @param other other instance of {@link Vector}
     * @return
     */
    public float dotProduct(Point other);

    /**
     * Modifies the length of the vector to be module = 1.
     */
    public void normalize();

    public float getX();

    public float getY();

    public float getZ();

}

package es.usc.citius.lab.motionplanner.core.spatial;

/**
 *
 * @since 21/11/2018
 * @param <C> type of the instance to rotate
 */
public interface Rotable<C> {

    /**
     * Rotates the spatial instance given the Euler angles.
     *
     * @param yaw angle around Z
     * @param pitch angle around Y
     * @param roll angle around X
     * @return rotated instance
     */
    public C rotate(float yaw, float pitch, float roll);

}

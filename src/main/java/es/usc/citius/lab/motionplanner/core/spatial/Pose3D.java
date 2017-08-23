package es.usc.citius.lab.motionplanner.core.spatial;

/**
 * Class defining a pose in 3D, where the location (x, y z) and heading (roll, pitch, yaw)
 * of an object is given.
 *
 * @author Adrián González Sieira <<a href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>>
 */
public class Pose3D extends Point3D{

    public float yaw, pitch, roll;

    /**
     * Constructor specifying all values independently.
     *
     * @param x
     * @param y
     * @param z
     * @param yaw
     * @param pitch
     * @param roll
     */
    public Pose3D(float x, float y, float z, float yaw, float pitch, float roll) {
        super(x, y, z);
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
    }

    /**
     * Constructor from a {@link Point3D}, specifying also the heading.
     *
     * @param other position in 3D space, instance of {@link Point3D}
     * @param yaw rotation around Z
     * @param pitch rotation around Y
     * @param roll rotation around X
     */
    public Pose3D(Point3D other, float yaw, float pitch, float roll) {
        super(other);
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
    }

    /**
     * Constructor to generate a copy of a {@link Pose3D}
     *
     * @param other other instance of this class
     */
    public Pose3D(Pose3D other) {
        this(other.x, other.y, other.z, other.yaw, other.pitch, other.roll);
    }

    public float getYaw() {
        return yaw;
    }

    public float getPitch() {
        return pitch;
    }

    public float getRoll() {
        return roll;
    }
}

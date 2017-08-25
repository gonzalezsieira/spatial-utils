/**
 * Copyright (C) 2014-2017 Adrián González Sieira (adrian.gonzalez@usc.es)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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

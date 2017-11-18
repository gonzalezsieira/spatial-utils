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

import es.usc.citius.lab.motionplanner.core.util.MathFunctions;

import java.io.Serializable;

/**
 * Class defining a pose in 3D, where the location (x, y z) and heading (roll, pitch, yaw)
 * of an object is given.
 *
 * @author Adrián González Sieira <<a href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>>
 */
public class Pose3D extends Point3D implements Pose, Serializable{

    public static final Pose3D ZERO = new Pose3D(Point3D.ZERO, 0,0, 0);
    private static final long serialVersionUID = 20171003L;

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

    /************************************************************************
     *                      	GETTERS
     ************************************************************************/

    public float getYaw() {
        return yaw;
    }

    public float getPitch() {
        return pitch;
    }

    public float getRoll() {
        return roll;
    }

    public static float[][] rotateXYZCoordinates(float x, float y, float z, float yaw, float pitch, float roll, float rotateYaw, float rotatePitch, float rotateRoll){
        float[] coordinates = Point3D.rotateXYZCoordinates(x, y, z, rotateYaw, rotatePitch, rotateRoll);
        float[] angles = new float[]{
            MathFunctions.adjustAngleP(yaw + rotateYaw),
            Math.abs(MathFunctions.adjustAngleP(pitch + rotatePitch)),
            MathFunctions.adjustAngleP(roll + rotateRoll)
        };
        return new float[][]{coordinates, angles};
    }

    /**
     * Rotates the position and heading of the pose
     *
     * @param roll rotation in roll
     * @param pitch rotation in pitch
     * @param yaw rotation in yaw
     * @return rotated state (position and heading)
     */
    public Pose3D rotate(float yaw, float pitch, float roll){
        float[][] rotation = Pose3D.rotateXYZCoordinates(this.x, this.y, this.z, this.yaw, this.pitch, this.roll, yaw, pitch, roll);
        //keep velocities
        return new Pose3D(rotation[0][0], rotation[0][1], rotation[0][2], rotation[1][0], rotation[1][1], rotation[1][2]);
    }

    @Override
    public void staticRotate(float yaw, float pitch, float roll) {
        float[][] rotation = rotateXYZCoordinates(this.x, this.y, this.z, this.yaw, this.pitch, this.roll, yaw, pitch, roll);
        this.x = rotation[0][0];
        this.y = rotation[0][1];
        this.z = rotation[0][2];
        this.yaw = rotation[1][0];
        this.pitch = rotation[1][1];
        this.roll = rotation[1][2];
    }

    /**
     * Performs the sum of the (x, y, z) coordinates
     *
     * @param move transform point
     * @return new {@link Pose3D} with coordinates (pose.x + move.x, pose.y + move.y, pose.z + move.z, pose.yaw, pose.pitch, pose.roll)
     */
    @Override
    public Pose3D add(Point move) {
        return new Pose3D(this.x + move.getX(), this.y + move.getY(), this.z + move.getZ(), this.yaw, this.pitch, this.roll);
    }

    /**
     * Performs the subtraction of the (x, y, z) coordinates
     *
     * @param move transform point
     * @return new {@link Point3D} with coordinates (pose.x - move.x, pose.y - move.y, pose.z - move.z, pose.yaw, pose.pitch, pose.roll)
     */
    @Override
    public Pose3D subtract(Point move) {
        return new Pose3D(this.x - move.getX(), this.y - move.getY(), this.z - move.getZ(), this.yaw, this.pitch, this.roll);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        if (!super.equals(o)) return false;

        Pose3D pose3D = (Pose3D) o;

        if (Math.round(yaw * PRECISION) != Math.round(pose3D.yaw * PRECISION)) return false;
        if (Math.round(pitch * PRECISION) != Math.round(pose3D.pitch * PRECISION)) return false;
        return Math.round(roll * PRECISION) == Math.round(pose3D.roll * PRECISION);
    }

    @Override
    public int hashCode() {
        int result = super.hashCode();
        result = 31 * result + Math.round(yaw * PRECISION);
        result = 31 * result + Math.round(pitch * PRECISION);
        result = 31 * result + Math.round(roll * PRECISION);
        return result;
    }
}

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

import java.io.Serializable;

import es.usc.citius.lab.motionplanner.core.util.MathFunctions;

import org.apache.commons.math3.util.FastMath;
import org.ejml.data.DenseMatrix64F;

/**
 * Extends the {@link Point2D} class to add information about the
 * pose (which includes the heading, or yaw angle).
 *
 * @author Adrián González Sieira <<a href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>>
 */
public class Pose2D extends Point2D implements Pose, Serializable{

    public static final Pose2D ZERO = new Pose2D(Point2D.ZERO, 0);
    private static final long serialVersionUID = 20140710L;
    
    public float yaw;

    /**
     * Constructor that builds this class with the information about the point (x, y) and the
     * angle defining the heading (yaw).
     *
     * @param x first coordinate
     * @param y second coordinate
     * @param yaw heading
     */
    public Pose2D(float x, float y, float yaw) {
        super(x, y);
        this.yaw = yaw;
    }

    /**
     * Constructor that takes a {@link Point2D} instance and builds a {@link Pose2D} with information
     * about the heading.
     *
     * @param other information about (x, y)
     * @param yaw heading
     */
    public Pose2D(Point2D other, float yaw) {
        this(other.x, other.y, yaw);
    }
    
    /**
     * Constructor that takes a {@link Pose2D} instance and builds a copy of its information.
     * 
     * @param other instance to copy
     */
    public Pose2D(Pose2D other){
    	this(other.x, other.y, other.yaw);
    }

    @Override
    public String toString() {
        return "[x=" + x + ", y=" + y + ", yaw=" + yaw + "]";
    }

    /************************************************************************
     *                              GETTERS
     ************************************************************************/

    @Override
    public float getYaw() {
        return yaw;
    }

    @Override
    public float getPitch() {
        return 0;
    }

    @Override
    public float getRoll() {
        return 0;
    }

    @Override
    public DenseMatrix64F getMatrix() {
        return new DenseMatrix64F(new double[][]{{x}, {y}, {yaw}});
    }

    /************************************************************************
     * 			GEOMETRIC OPERATION METHODS
     ************************************************************************/

    /**
     * Obtains the relative yaw angle (in the plane X-Y) between the current
     * heading and other point in the plane X-Y.
     *
     * @param point in the X-Y plane
     * @return relative angle to the point, adjusted to (-PI, PI]
     */
    public float relativeYawTo(Point2D point){
    	return MathFunctions.adjustAngleP((float) FastMath.atan2(point.y - y, point.x - x) - yaw);
    }

    /**
     * Obtains the symmetric state respect to the X axis:
     * (x, -y, reflectedYawX, vx, -w)
     *
     * @return reflected {@link State2D}, respect to the X axis
     */
    public Pose2D symmetricPlaneXZ(){
        float angle = MathFunctions.adjustAngleP(-yaw);
        return new Pose2D(x, -y, angle);
    }

    /**
     * Obtains the symmetric state respect to the Y axis:
     * (-x, y, reflectedYawY, vx, -w)
     *
     * @return reflected {@link State2D}, respect to the Y axis
     */
    public Pose2D symmetricPlaneYZ(){
        float angle = MathFunctions.adjustAngleP(MathFunctions.PI - yaw);
        return new Pose2D(-x, y, angle);
    }

    public Pose2D symmetricPlaneXY() {
        return new Pose2D(this);
    }

    /************************************************************************
     *                       EQUALS/HASH-CODE METHODS
     ************************************************************************/
	
    @Override
    public int hashCode() {
        final int prime = 31;
        int result = super.hashCode();
        result = prime * result + Math.round(yaw * PRECISION);
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
                return true;
        if (getClass() != obj.getClass())
                return false;
        Pose2D other = (Pose2D) obj;
        if (Math.round(yaw * PRECISION) != Math.round(other.yaw * PRECISION))
                return false;
        if (!super.equals(obj))
                return false;

        return true;
    }

    @Override
    public void staticRotate(float yaw, float pitch, float roll) {
        super.staticRotate(yaw, pitch, roll);
        //rotated heading
        this.yaw = MathFunctions.adjustAngleP(this.yaw + yaw);
    }

    @Override
    public Pose2D rotate(float yaw, float pitch, float roll) {
        //rotated (x, y)
        float[] rotatedXY = Point2D.rotateXYCoordinates(this.x, this.y, yaw);
        //rotated heading
        float newAngle = MathFunctions.adjustAngleP(this.yaw + yaw);
        //new State2D
        return new Pose2D(rotatedXY[0], rotatedXY[1], newAngle);
    }

    /**
     * Performs the sum of the (x, y) coordinates
     *
     * @param move transform point
     * @return new {@link Pose2D} with coordinates (point.x + move.x, point.y + move.y, point.yaw)
     */
    @Override
    public Pose2D add(Point move) {
        return new Pose2D(this.x + move.getX(), this.y + move.getY(), this.yaw);
    }

    /**
     * Performs the subtraction of the (x, y) coordinates
     *
     * @param move transform point
     * @return new {@link Pose2D} with coordinates (point.x - move.x, point.y - move.y, point.yaw)
     */
    @Override
    public Pose2D subtract(Point move) {
        return new Pose2D(this.x - move.getX(), this.y - move.getY(), this.yaw);
    }

}

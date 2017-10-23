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

import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;

/**
 * Extends the {@link Pose2D} to add information of the 
 * the linear speed (vx) and angular speed (w).
 *
 * @author Adrián González Sieira <<a href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>>
 */
public class State2D extends Pose2D implements Serializable, Geometry<State2D> {
	
    public static final State2D ZERO = new State2D(Pose2D.ZERO, 0,0, 0);
    private static final long serialVersionUID = 20140710L;
    
    public float vx;
    public float vy;
    public float w;

    /**
     * Builds a {@link State2D} instance from a {@link Pose2D} instance
     * and the linear and angular speeds.
     *
     * @param vx current linear speed
     * @param w current angular speed
     * @param pose current 2D pose
     */
    public State2D(Pose2D pose, float vx, float vy, float w) {
    	super(pose);
        this.vx = vx;
        this.vy = vy;
        this.w = w;
    }

    /**
     * Builds a {@link State2D} instance with the data of the
     * Position instance, and the linear and angular speed.
     *
     * @param vx current linear speed
     * @param w current angular speed
     * @param x current X position
     * @param y current Y position
     * @param yaw current heading
     */
    public State2D(float x, float y, float yaw, float vx, float vy, float w) {
        super(x, y, yaw);
        this.vx = vx;
        this.vy = vy;
        this.w = w;
    }
    
    /**
     * Given a column matrix 1x5, maps the corresponding robot state
     *
     * @param matrix a column matrix 1x6
     */
    public State2D(DenseMatrix64F matrix) {
        this(
            (float) matrix.get(0, 0),
            (float) matrix.get(1, 0),
            (float) matrix.get(2, 0),
            (float) matrix.get(3, 0),
            (float) matrix.get(4, 0),
            (float) matrix.get(5, 0)
        );
    }

    /**
     * Given a column matrix 1x5, maps the corresponding robot state
     *
     * @param matrix a column matrix 1x5
     * @param vx linear velocity
     * @param w angular velocity
     */
    public State2D(SimpleMatrix matrix, float vx, float vy, float w) {
        this(
                (float) matrix.get(0, 0),
                (float) matrix.get(1, 0),
                (float) matrix.get(2, 0),
                vx,
                vy,
                w
        );
    }
    
    /**
     * Copies the information of other instance to create a new one.
     * 
     * @param other 
     */
    public State2D(State2D other){
        this(other.x, other.y, other.yaw, other.vx, other.vy, other.w);
    }
    
    @Override
    public String toString() {
    	return "[x=" + x + ", y=" + y + ", yaw=" + yaw + ", vx=" + vx + ", vy=" + vy + ", w=" + w + "]";
    }

    @Override
    public int hashCode() {
        int prime = 31;
        int result = super.hashCode();
        result = prime * result + Math.round(vx * 1E+4f);
        result = prime * result + Math.round(vy * 1E+4f);
        result = prime * result + Math.round(w * 1E+4f);
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final State2D other = (State2D) obj;
        if (Math.abs(vx - other.vx) > 1E-4f) {
            return false;
        }
        if (Math.abs(vy - other.vy) > 1E-4f) {
            return false;
        }
        if (Math.abs(w - other.w) > 1E-4f) {
            return false;
        }
        if (!super.equals(obj))
            return false;
        return true;
    }
    
    
    /************************************************************************
     *                      	GETTERS
     ************************************************************************/

    public float getW() {
        return w;
    }

    public float getVx() {
        return vx;
    }

    public float getVy() { return vy; }

    /**
     * Builds a {@link SimpleMatrix} with the state values: (x, y, yaw, vx, vy, w)
     *
     * @return a 6x1 {@link SimpleMatrix}
     */
    @Override
    public DenseMatrix64F getMatrix() {
        return new DenseMatrix64F(new double[][]{{x}, {y}, {yaw}, {vx}, {vy}, {w}});
    }

    /************************************************************************
     * 					GEOMETRIC OPERATION METHODS
     ************************************************************************/

    /**
     * Obtains the symmetric state respect to the X axis:
     * (x, -y, reflectedYawX, vx, -w)
     * 
     * @return reflected {@link State2D}, respect to the X axis
     */
    @Override
    public State2D symmetricPlaneXZ(){
        float angle = MathFunctions.adjustAngleP(-yaw);
        return new State2D(x, -y, angle, vx, -vy, -w);
    }

    /**
     * Obtains the symmetric state respect to the Y axis:
     * (-x, y, reflectedYawY, vx, -w)
     * 
     * @return reflected {@link State2D}, respect to the Y axis
     */
    @Override
    public State2D symmetricPlaneYZ(){
        float angle = MathFunctions.adjustAngleP(MathFunctions.PI - yaw);
        return new State2D(-x, y, angle, vx, -vy, -w);
    }

    @Override
    public State2D symmetricPlaneXY() {
        return new State2D(this);
    }

    /************************************************************************
     *                 	STATIC GEOMETRIC OPERATION METHODS
     ************************************************************************/

    /**
     * Performs the sum of the (x, y) coordinates of two instances
     * of {@link Point2D}. 
     *
     * @param move adding point
     * @return new {@link State2D} with coordinates (this.x + move.x, this.y + move.y, this.yaw, this.vx, this.vy, this.w)
     */
    @Override
    public State2D add(Point3D move){
        return new State2D(this.x + move.x, this.y + move.y, this.yaw, this.vx, this.vy, this.w);
    }

    public State2D add(Point2D move) {
        return new State2D(this.x + move.x, this.y + move.y, this.yaw, this.vx, this.vy, this.w);
    }

    /**
     * Performs the subtract of the (x, y) coordinates of a {@link State2D} and
     * a {@link Point2D}.
     *
     * @param move subtracting point
     * @return new {@link State2D} with coordinates (this.x - move.x, this.y - move.y, this.yaw, this.vx, this.vy, this.w)
     */
    @Override
    public State2D subtract(Point3D move){
        return new State2D(this.x - move.x, this.y - move.y, this.yaw, this.vx, this.vy, this.w);
    }

    public State2D subtract(Point2D move) {
        return new State2D(this.x - move.x, this.y - move.y, this.yaw, this.vx, this.vy, this.w);
    }

    /**
    * Rotates the an instance of {@link Point2D} and obtains the 
    * point rotated by the angle specified.
    *
    * @param yaw rotation angle
    * @return new {@link Point2D} after rotation
    */
    @Override
    public State2D rotate(float yaw, float pitch, float roll){
        //rotated (x, y)
        float[] rotatedXY = Point2D.rotateXYCoordinates(this.x, this.y, yaw);
        //rotated heading
        float newAngle = MathFunctions.adjustAngleP(this.yaw + yaw);
        //new State2D
        return new State2D(rotatedXY[0], rotatedXY[1], newAngle, this.vx, this.vy, this.w);
    }

    @Override
    public State2D clone() {
        return new State2D(this);
    }

    @Override
    public int symmetryPlane() {
        //obtain angle / (pi / 2) mod 2. If angleSymmetriAxis == 0 then Axis = X, else Axis = Y
        return Math.abs(Math.round( 2 * this.absoluteAngle() / MathFunctions.PI)) % 2;
    }
}

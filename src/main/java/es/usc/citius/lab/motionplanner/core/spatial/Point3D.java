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
import org.ejml.simple.SimpleMatrix;

/**
 * Class defining a point in the 3D-space.
 *
 * @author Adrián González Sieira <adrian.gonzalez@usc.es>
 */
public class Point3D implements Serializable{

    public static final Point3D ZERO = new Point3D(0, 0, 0);
    private static final long serialVersionUID = 20140710L;
    
    public float x, y, z;

	protected static final float PRECISION = 1E+4f;

    /**
     * Constructor that initializes a 3D point from its coordinates.
     * 
     * @param x
     * @param y
     * @param z
     */
    public Point3D(float x, float y, float z){
            this.x = x;
            this.y = y;
            this.z = z;
    }

    /**
     * Constructor that clones a {@link Point3D} existing object.
     * 
     * @param other instance to copy
     */
    public Point3D(Point3D other){
            this(other.x, other.y, other.z);
    }

    /**
     * Constructor that transforms a {@link Point2D} in a {@link Point3D},
     * with z=0.
     *
     * @param point2d 2D instance
     */
    public Point3D(Point2D point2d){
        this(point2d.x, point2d.y, 0);
    }
	
    @Override
    public String toString() {
            return "[x=" + x + ", y=" + y + ", z=" + z + "]";
    }

    /**
     * Retrieves a {@link SimpleMatrix} with the information of this point.
     * 
     * @return 
     */
    public DenseMatrix64F getMatrix() {
        return new DenseMatrix64F(new double[][]{{x}, {y}, {z}});
    }

    /************************************************************************
     * 								GETTERS
     ************************************************************************/

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public float getZ() {
            return z;
    }

    /************************************************************************
     * 					EQUALS/HASH-CODE METHODS
     ************************************************************************/

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + Math.round(x * PRECISION);
        result = prime * result + Math.round(y * PRECISION);
        result = prime * result + Math.round(z * PRECISION);
        return result;
    }

    @Override
    public boolean equals(Object obj) {

        if (this == obj) {
            return true;
        }
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        Point3D other = (Point3D) obj;
        if (Math.round(x * PRECISION) != Math.round(other.x * PRECISION)) {
            return false;
        }
        if (Math.round(y * PRECISION) != Math.round(other.y * PRECISION)) {
            return false;
        }
        if (Math.round(z * PRECISION) != Math.round(other.z * PRECISION)) {
            return false;
        }
        return true;
    }

    /**
     * Measures the angle around Z axis (yaw) between two 3D points.
     *
     * @param point
     * @return yaw
     */
    public float yawTo(Point3D point){
        return MathFunctions.adjustAngleP((float) FastMath.atan2(point.y - y, point.x - x));
    }

    /**
     * Measures the angle around Y (pitch) between two 3D points.
     *
     * @param point
     * @return pitch
     */
    public float pitchTo(Point3D point){
        return MathFunctions.adjustAngleP((float) FastMath.atan2(point.z - z, point.x - x));
    }

    /**
     * Measures the angle around X (roll) between two 3D points.
     *
     * @param point
     * @return roll
     */
    public float rollTo(Point3D point){
        return MathFunctions.adjustAngleP((float) FastMath.atan2(point.y - y, point.z - z));
    }

    /**
     * Obtains euclidean distance between two {@link Point3D}.
     * @param point
     *
     * @return euclidean distance
     */
    public float distance(Point3D point) {
        return (float) FastMath.sqrt(FastMath.pow(point.x - this.x, 2) + FastMath.pow(point.y - this.y, 2) + FastMath.pow(point.z - this.z, 2));
    }

    /**
     * Calculates the projection of this point over a line segment.
     *
     * @param S1 first point of the segment
     * @param S2 second point of the segment
     * @return closest point over the segment.
     */
    public Point3D projectionToSegment(Point3D S1, Point3D S2) {

        //S2 - S1
        Vector3D v = new Vector3D(S1, S2);
        //P - S1
        Vector3D w = new Vector3D(S1, this);

        //W·V
        float c1 = w.dotProduct(v);
        //S1 is the closest point
        if (c1 <= 0)
            return S1;

        //V·V
        float c2 = v.dotProduct(v);
        //S2 is the closest point
        if (c2 <= c1)
            return S2;

        //if in the middle of the segment, obtain distance to projection
        float b = c1 / c2;
        //projection
        return new Point3D(S1.x + b * v.x, S1.y + b * v.y, S1.z + b * v.z);

    }

    /**
     * Calculates the distance between the point and a line segment.
     *
     * @param S1 first point of the segment
     * @param S2 second point of the segment
     * @return shortest distance between the point and the segment
     */
    public float distanceToSegment(Point3D S1, Point3D S2){

        //S2 - S1
        Vector3D v = new Vector3D(S1, S2);
        //P - S1
        Vector3D w = new Vector3D(S1, this);

        //W·V
        float c1 = w.dotProduct(v);
        //S1 is the closest point
        if(c1 <= 0)
            return this.distance(S1);

        //V·V
        float c2 = v.dotProduct(v);
        //S2 is the closest point
        if(c2 <= c1)
            return this.distance(S2);

        //if in the middle of the segment, obtain distance to projection
        float b = c1 / c2;
        //projection
        Point3D B = new Point3D(S1.x + b * v.x, S1.y + b * v.y, S1.z + b * v.z);
        return this.distance(B);

    }

    /**
     * **********************************************************************
     * GEOMETRIC OPERATION METHODS
     * **********************************************************************
     */
    /**
     * Rotates this instance of {@link Point3D} according to the
     * specified angles.
     * 
     * @param yaw rotation in the XY plane
     * @param pitch rotation in the XZ plane
     * @param roll rotation in the YZ plane
     */
    public void rotate(float yaw, float pitch, float roll){
        float[] rotated = rotateXYZCoordinates(x, y, z, yaw, pitch, roll);
        x = rotated[0];
        y = rotated[1];
        z = rotated[2];
    }

    /**
     * Adds a point to the current one.
     * @param point
     */
    public void add(Point3D point){
        x += point.x;
        y += point.y;
        z += point.z;
    }
    
    /**
     * **********************************************************************
     * STATIC GEOMETRIC OPERATION METHODS
     * **********************************************************************
     */
    /**
     * Performs the sum of the (x, y, z) coordinates of two instances of
     * {@link Point3D}.
     *
     * @param point original point
     * @param move transform point
     * @return new {@link Point3D} with coordinates (point.x + move.x, point.y + move.y, point.z + move.z)
     */
    public static Point3D add(Point3D point, Point3D move) {
        return new Point3D(point.x + move.x, point.y + move.y, point.z + move.z);
    }

    /**
     * Rotates this instance of {@link Point3D} according to the
     * specified angles, returning a new instance.
     * 
     * @param point source instance of {@link Point3D} 
     * @param yaw rotation in the XY plane
     * @param pitch rotation in the XZ plane
     * @param roll rotation in the YZ plane
     * @return rotated instance of {@link Point3D}
     */
    public static Point3D rotate(Point3D point, float yaw, float pitch, float roll){
        float[] rotated = rotateXYZCoordinates(point.x, point.y, point.z, yaw, pitch, roll);
        return new Point3D(rotated[0], rotated[1], rotated[2]);
    }
    
    /**
     * Implements the rotation for the x-y coordinates and returns an array with
     * the rotated ones; see 3D rotation angles and matrix in LaValle:
     * http://planning.cs.uiuc.edu/node101.html#fig:yawpitchroll
     * http://planning.cs.uiuc.edu/node102.html
     *
     * @param x
     * @param y
     * @param z 
     * @param yaw rotation in Z angle (positive, Y axis anticlockwise (left))
     * @param pitch rotation in Y angle (positive, X axis anticlockwise (down))
     * @param roll rotation in X angle (positive, Y axis clockwise (up))
     * @return float[xRotated, yRotated, zRotated] 
     */
    protected static float[] rotateXYZCoordinates(float x, float y, float z, 
            float yaw, float pitch, float roll) {
        
        //obtain values a-priori
        double sinyaw = FastMath.sin(yaw);
        double sinpitch = FastMath.sin(pitch);
        double sinroll = FastMath.sin(roll);
        double cosyaw = FastMath.cos(yaw);
        double cospitch = FastMath.cos(pitch);
        double cosroll = FastMath.cos(roll);
        
        return new float[]{
            (float) (x * (cosyaw * cospitch) 
                + y * (-sinyaw * cosroll + cosyaw * sinpitch * sinroll) 
                + z * (sinyaw * sinroll + cosyaw * sinpitch * cosroll)),
            (float) (x * (sinyaw * cospitch)
                + y * (cosyaw * cosroll + sinyaw * sinpitch * sinroll)
                + z * (-cosyaw * sinroll + sinyaw * sinpitch * cosroll)),
            (float) (x * (-sinpitch)
                + y * (cospitch * sinroll)
                + z * (cospitch * cosroll))
        };
    }
	
}

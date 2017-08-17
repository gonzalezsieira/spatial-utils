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

import org.apache.commons.math3.util.FastMath;

import es.usc.citius.lab.motionplanner.core.util.MathFunctions;
import java.io.Serializable;

import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;

/**
 * Implements a 2D point (x, y) in the space.
 *
 * @author Adrián González Sieira <adrian.gonzalez@usc.es>
 */
public class Point2D implements Serializable{

    public static final Point2D ZERO = new Point2D(0, 0);
    private static final long serialVersionUID = 20140710L;
    
    public float x;
    public float y;

    protected static final float PRECISION = 1E+4f;

    /**
     * Default constructor for this class.
     *
     * @param x
     * @param y
     */
    public Point2D(float x, float y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Constructor that creates a new instance based on the information of
     * another.
     *
     * @param other instance to clone
     */
    public Point2D(Point2D other) {
        this(other.x, other.y);
    }

    @Override
    public String toString() {
        return "[x=" + x + ", y=" + y + "]";
    }

    /**
     * **********************************************************************
     * GETTERS
     * **********************************************************************
     */
    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }
    
    /**
     * Retrieves a {@link SimpleMatrix} with the information of this point.
     * 
     * @return a 1x2 matrix
     */
    public DenseMatrix64F getMatrix(){
        return new DenseMatrix64F(new double[][]{{x}, {y}});
    }

    /**
     * **********************************************************************
     * EQUALS/HASH-CODE METHODS
     * **********************************************************************
     */
    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + Math.round(x * PRECISION);
        result = prime * result + Math.round(y * PRECISION);
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
        Point2D other = (Point2D) obj;
        if (Math.round(x * PRECISION) != Math.round(other.x * PRECISION)) {
            return false;
        }
        if (Math.round(y * PRECISION) != Math.round(other.y * PRECISION)) {
            return false;
        }
        return true;
    }

    /**
     * **********************************************************************
     * GEOMETRIC OPERATION METHODS
     * **********************************************************************
     */
    /**
     * Obtains the absolute angle between this point and the origin:
     * {@code atan2(y, x)}
     *
     * @return absolute angle in the interval (-PI, PI]
     */
    public float absoluteAngle() {
        return MathFunctions.adjustAngleP((float) FastMath.atan2(y, x));
    }

    /**
     * Obtains the absolute angle between this point and other point acting as
     * center of coordinates: {@code atan2(y - point.y, x - point.x)}
     *
     * @param point other instance of {@link Point2D}
     * @return absolute angle in the interval (-PI, PI]
     */
    public float absoluteAngleTo(Point2D point) {
        return MathFunctions.adjustAngleP((float) FastMath.atan2(point.y - y, point.x - x));
    }

    /**
     * Obtains the scalar distance between this point and other {@link Point2D}.
     *
     * @param point reference point
     * @return scalar distance between them
     */
    public float distance(Point2D point) {
        return (float) FastMath.hypot(x - point.x, y - point.y);
    }

    /**
     * Adds the coordinates of this point and other instance of {@link Point2D}.
     *
     * @param point other instance of {@link Point2D}
     */
    public void add(Point2D point) {
        x += point.x;
        y += point.y;
    }

    /**
     * Subtracts to the coordinates of this point the coordinates of other
     * {@link Point2D}.
     *
     * @param point other instance of {@link Point2D}
     */
    public void subtract(Point2D point) {
        x -= point.x;
        y -= point.y;
    }

    /**
     * Applies a rotation over the coordinates of this point.
     *
     * @param angle in interval (-PI, PI]
     */
    public void rotate(float angle) {
        float[] rotated = rotateXYCoordinates(x, y, angle);
        x = rotated[0];
        y = rotated[1];
    }

    /**
     * Calculates the projection of the current instance over the line defined
     * by the two points r1 and r2.
     *
     * @param r1 first point of the line
     * @param r2 second point of the line
     * @return projection of the point over the line
     */
    public Point2D projectOverLine(Point2D r1, Point2D r2) {
        //aux variables to avoid repeating operations
        float dx = r2.x - r1.x;
        float dy = r2.y - r1.y;
        float r2xsq = r2.x * r2.x;
        float r2ysq = r2.y * r2.y;
        float r1xsq = r1.x * r1.x;
        float r1ysq = r1.y * r1.y;
        float xdxydy = x * dx + y * dy;
        float r1ydxr1xdy = r1.y * dx - r1.x * dy;
        //obtain values
        float denom = 1.0f / (r2xsq - 2.0f * r2.x * r1.x + r1xsq - 2.0f * r1.y * r2.y + r1ysq + r2ysq);
        float numx = -(-dx * xdxydy) + (-dy * r1ydxr1xdy);
        float numy = -(-dy * xdxydy) - (-dx * r1ydxr1xdy);
        //return instance
        return new Point2D(numx * denom, numy * denom);
    }
    
    /**
     * Calculates the projection of the current instance over the segment defined
     * by the two points r1 and r2; projected point is always within the borders
     * of the segment.
     * 
     * @param r1 first point of the line
     * @param r2 second point of the line
     * @return projection of the point over the segment
     * 
     * @see {@url http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment}
     */
    public Point2D projectOverSegment(Point2D r1, Point2D r2){
        float dx = r2.x - r1.x; float dy = r2.y - r1.y;
        float length2 = dx * dx + dy * dy;
        float t = ((this.x - r1.x) * dx + (this.y - r1.y) * dy) / length2;
        //point beyond the first point of the segment
        if(t < 0 ) return r1;
        //point beyond the second point of the segment
        if(t > 1) return r2;
        //point between r1 and r2
        return new Point2D(r1.x + t * dx, r1.y + t * dy);
    }

    /**
     * **********************************************************************
     * STATIC GEOMETRIC OPERATION METHODS
     * **********************************************************************
     */
    /**
     * Performs the sum of the (x, y) coordinates of two instances of
     * {@link Point2D}.
     *
     * @param point original point
     * @param move transform point
     * @return new {@link Point2D} with coordinates (point.x + move.x, point.y +
     * move.y)
     */
    public static Point2D add(Point2D point, Point2D move) {
        return new Point2D(point.x + move.x, point.y + move.y);
    }

    /**
     * Performs the subtract of the (x, y) coordinates of two instances of
     * {@link Point2D}.
     *
     * @param point original point
     * @param move transform point
     * @return new {@link Point2D} with coordinates (point.x - move.x, point.y -
     * move.y)
     */
    public static Point2D subtract(Point2D point, Point2D move) {
        return new Point2D(point.x - move.x, point.y - move.y);
    }

    /**
     * Rotates the an instance of {@link Point2D} and obtains the point rotated
     * by the angle specified.
     *
     * @param point current (x, y) point
     * @param angle rotation angle
     * @return new {@link Point2D} after rotation
     */
    public static Point2D rotate(Point2D point, float angle) {
        //new (x, y)
        float[] rotatedXY = rotateXYCoordinates(point.x, point.y, angle);
        return new Point2D(rotatedXY[0], rotatedXY[1]);
    }

    /**
     * Implements the rotation for the x-y coordinates and returns an array with
     * the rotated ones.
     *
     * @param x
     * @param y
     * @param angle
     * @return float[xRotated, yRotated]
     */
    protected static float[] rotateXYCoordinates(float x, float y, float angle) {
        //obtain the a-priori
        float cos = (float) FastMath.cos(angle);
        float sin = (float) FastMath.sin(angle);
        //calculate new point
        return new float[]{
            x * cos - y * sin,
            x * sin + y * cos
        };
    }
}

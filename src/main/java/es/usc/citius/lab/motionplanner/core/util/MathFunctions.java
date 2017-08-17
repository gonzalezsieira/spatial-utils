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
package es.usc.citius.lab.motionplanner.core.util;

import org.apache.commons.math3.util.FastMath;

/**
 * Collection of mathematical methods related to angles and distances.
 *
 * @author Adrián González Sieira <adrian.gonzalez@usc.es>
 */
public abstract class MathFunctions {

	
    public static final float PI = (float) FastMath.PI;
    public static final float PITIMES2 = (float) (2 * FastMath.PI);
    public static final float INV_PITIMES2 = (float) (1 / PITIMES2);
    public static final float PIDIV2 = (float) (FastMath.PI / 2);
    public static final float PIDIV4 = (float) (FastMath.PI / 4);


    /**
     * Private method to implement the singleton pattern.
     */
    private MathFunctions() {
    }

    /**
     * This function adjusts an input angle into the range [0, 2*PI).
     *
     * @param angle input value of the angle, in radians
     * @return an output value adapted to the angle range
     */
    public static float adjustAngle2P(float angle) {
        float angleRange = angle;
    	while(angleRange < 0){
        	angleRange += PITIMES2;
        }
    	while(angleRange >= PITIMES2){
    		angleRange -= PITIMES2;
    	}
    	return angleRange;
    }

    /**
     * This function adjusts an input angle into the range (-PI, PI].
     *
     * @param angle input value of the angle, in radians
     * @return an output value adapted to the angle range
     */
    public static float adjustAngleP(float angle) {
        float angleRange = angle;
    	while(angleRange <= -PI){
        	angleRange += PITIMES2;
        }
    	while(angleRange > PI){
    		angleRange -= PITIMES2;
    	}
    	return angleRange;
    }

    /**
     * This method converts an angle expressed in degrees, into the equivalent
     * one on radians.
     *
     * @param angle input angle in degrees
     * @return output angle converted to radians
     */
    public static float degToRadians(float angle) {
        return (float) FastMath.toRadians(angle);
    }

    /**
     * This method converts an angle expressed in radians, into the equivalent
     * on degrees.
     *
     * @param angle input angle in radians
     * @return output angle converted in degrees
     */
    public static float radiansToDeg(float angle) {
        return (float) FastMath.toDegrees(angle);
    }

    /**
     * This method calculates the distance between two points of the 2D space
     * determined by their coordinates X and Y.
     *
     * @param x Coordinate x of the first point
     * @param y Coordinate y of the first point
     * @param x2 Coordinate x of the second point
     * @param y2 Coordinate y of the second point
     * @return The distance value between the two points
     */
    public static float distanceBetweenPointsFloat(float x, float y, float x2, float y2) {
        return (float) FastMath.sqrt(FastMath.pow(x2 - x, 2) + FastMath.pow(y2 - y, 2));
    }

    public static double distanceBetweenPointsDouble(float x, float y, float x2, float y2) {
        return FastMath.sqrt(FastMath.pow(x2 - x, 2) + FastMath.pow(y2 - y, 2));
    }

    /**
     * This method returns the signed difference between two angles, adjusted in
     * the interval between positive pi and negative pi.
     *
     * @param angle1 angle of the beginning position
     * @param angle2 angle of the ending position
     * @return the difference between them
     */
    public static float errorBetweenAngles(float angle1, float angle2) {
        return adjustAngleP(angle2 - angle1);
    }

    public static float trunc(float n, int d) {
        double factorPrecision = FastMath.pow(10, d);
        return (float) (FastMath.round(n * factorPrecision) / factorPrecision);
    }
    
    /**
     * Rounds an angle in radians to a degree precision equivalent.
     * 
     * @param angle angle in radians
     * @return angle adjusted to precision of 1º, in radians
     */
    public static float roundAngleToDegreePrecision(float angle){
    	return degToRadians((int) radiansToDeg(angle));
    }


    /**
     * Fast code to calculate the inverse of a square root.
     * This is used to avoid calculating the square root, always a method which is slower.
     *
     * @see <url>http://en.wikipedia.org/wiki/Fast_inverse_square_root</url>
     * @see <url>http://stackoverflow.com/questions/21010586/normalizing-spatial-vectors-without-square-root</url>
     * @see <url>http://stackoverflow.com/questions/11513344/how-to-implement-the-fast-inverse-square-root-in-java</url>
     * @param value float value to calculate 1/sqrt(value)
     * @return
     */
    public static float fastInverseSquareRootFloat(float value){
        float xhalf = 0.5f*value;
        int i = Float.floatToIntBits(value);
        i = 0x5f3759df - (i >> 1);
        value = Float.intBitsToFloat(i);
        value = value*(1.5f - xhalf*value*value);
        return value;
    }
}

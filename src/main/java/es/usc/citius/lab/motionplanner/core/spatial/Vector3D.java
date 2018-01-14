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
import org.apache.commons.math3.util.FastMath;

import java.io.Serializable;
import static es.usc.citius.lab.motionplanner.core.spatial.Point3D.rotateXYZCoordinates;

/**
 * Defines a 3D vector. A vector can be instantiated from two points, from
 * another vector, or manually given its three components.
 *
 * @author Adrián González Sieira <adrian.gonzalez@usc.es>
 */
public class Vector3D implements Serializable{

    private static final long serialVersionUID = 20140710L;
    public static final Vector3D X = new Vector3D(1f, 0f, 0f);
    public static final Vector3D Y = new Vector3D(0f, 1f, 0f);
    public static final Vector3D Z = new Vector3D(0f, 0f, 1f);
    public static final Vector3D ZERO = new Vector3D(0f, 0f, 0f);
    public float x;
    public float y;
    public float z;
    
    /**
     * Creates a vector [x, y, z] given its three elements.
     *
     * @param x first element
     * @param y second element
     * @param z third element
     */
    public Vector3D(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Computes the distance vector (p2 - p1).
     *
     * @param p1 origin
     * @param p2 destination
     */
    public Vector3D(Point3D p1, Point3D p2) {
        this.x = p2.x - p1.x;
        this.y = p2.y - p1.y;
        this.z = p2.z - p1.z;
    }
    
    /**
     * Dot product a·b, as detailed in 
     * http://en.wikipedia.org/wiki/Dot_product#Algebraic_definition.
     * 
     * @param other other point
     * @return dot product result
     */
    public float dotProduct(Vector3D other){
        return x*other.x + y*other.y + z*other.z;
    }
    
    /**
     * Dot product a·b, as detailed in 
     * http://en.wikipedia.org/wiki/Dot_product#Algebraic_definition,
     * but instead of using a {@link Vector3D} as second operand, takes the 
     * three parameters (x, y, z).
     * 
     * @param x first coordinate
     * @param y second coordinate
     * @param z third coordinate
     * @return 
     */
    public float dotProduct(float x, float y, float z){
        return this.x*x + this.y*y + this.z*z;
    }

    public float dotProduct(Point3D other) {
        return x*other.x + y*other.y + z*other.z;
    }

    public Vector3D rotate(float yaw, float pitch, float roll) {
        float[] rotated = rotateXYZCoordinates(this.x, this.y, this.z, yaw, pitch, roll);
        return new Vector3D(rotated[0], rotated[1], rotated[2]);
    }

    public void normalize(){
        float value = x * x + y * y + z * z;
        //avoids normalizing vectors when they are already unitary
        if(FastMath.abs(value) > 1.01f){
            float invLength = MathFunctions.fastInverseSquareRootFloat(value);
            this.x = x * invLength;
            this.y = y * invLength;
            this.z = z * invLength;
        }
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public float getZ() {
        return z;
    }
}

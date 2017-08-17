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

/**
 *
 *
 * @author Adrián González Sieira <<a href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>>
 * @since 04/05/2015
 */
public class Vector2D {

    public static final Vector2D X = new Vector2D(new Point2D(0f, 0f), new Point2D(1f, 0f));
    public static final Vector2D Y = new Vector2D(new Point2D(0f, 0f), new Point2D(0f, 1f));
    public float x;
    public float y;

    /**
     * Instantiate a new class Vector2D defining the a and b points of the segment.
     *
     * @param a
     * @param b
     */
    public Vector2D(Point2D a, Point2D b) {
        this.x = b.x - a.x;
        this.y = b.y - a.y;
    }

    public Vector2D(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public float dotProduct(Vector2D other){
        return this.x * other.x + this.y * other.y;
    }

    public float dotProduct(Point2D other){
        return this.x * other.x + this.y * other.y;
    }

    public void normalize(){
        float value = x * x + y * y;
        //avoids normalizing vectors when they are already unitary
        if(FastMath.abs(value) > 1.01f){
            float invLength = MathFunctions.fastInverseSquareRootFloat(value);
            this.x = x * invLength;
            this.y = y * invLength;
        }
    }
}

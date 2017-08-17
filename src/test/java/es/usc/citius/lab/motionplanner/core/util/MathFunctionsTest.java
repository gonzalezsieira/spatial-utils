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
import static es.usc.citius.lab.motionplanner.core.util.MathFunctions.*;
import org.junit.Test;

import junit.framework.TestCase;

/**
 * Tests if the functions implemented in {@link MathFunctions} are working
 * properly.
 *
 * @author Adrián González Sieira <adrian.gonzalez@usc.es>
 */
public class MathFunctionsTest extends TestCase {

    @Test
    public void test01_adjustAngleP() {
        for (float angleDegOutOfRange = -4 * 180; angleDegOutOfRange <= (4 * 180); angleDegOutOfRange += 0.01) {
            float angleOutOfRange = MathFunctions.degToRadians(angleDegOutOfRange);
            float angle1 = adjustAngleP_OldImpl(angleOutOfRange);
            float angle2 = MathFunctions.adjustAngleP(angleOutOfRange);
            assertEquals("Angles returned by the functions (-PI, PI] are different", angle1, angle2, 2 * Math.PI / 100000);
        }
    }
    
    @Test
    public void test01_adjustAngle2P() {
        for (float angleDegOutOfRange = -4 * 180; angleDegOutOfRange <= (4 * 180); angleDegOutOfRange += 0.01) {
            float angleOutOfRange = MathFunctions.degToRadians(angleDegOutOfRange);
            float angle1 = adjustAngle2P_OldImpl(angleOutOfRange);
            float angle2 = MathFunctions.adjustAngle2P(angleOutOfRange);
            assertEquals("Angles returned by the functions [0, 2PI) are different", angle1, angle2, 2 * Math.PI / 100000);
        }
    }

    /**
     * Old implementation of {@link MathFunctions#adjustAngleP(float) }
     *
     * @param angle
     * @return angle in (-PI, PI]
     */
    private float adjustAngleP_OldImpl(float angle) {
        //local variables
        float newAngle;
        float times;

        if (angle > PI) {
            times = FastMath.abs((float) FastMath.floor((angle + PI) * INV_PITIMES2));
            newAngle = angle - times * PITIMES2;
        } else if (angle <= -PI) {
            times = FastMath.abs((float) FastMath.ceil((angle - PI) * INV_PITIMES2));
            newAngle = angle + times * PITIMES2;
        } else {
            newAngle = angle;
        }

        //Min values of the angle are returned as the max one
        if (Float.compare(newAngle, -PI) == 0) {
            newAngle = PI;
        }
        return newAngle;
    }

    /**
     * Old implementation of {@link MathFunctions#adjustAngle2P(float) }
     * 
     * @param angle
     * @return angle in [0, 2PI)
     */
    public static float adjustAngle2P_OldImpl(float angle) {
        float newAngle;
        float times;

        if (angle >= PITIMES2) {
            times = (float) FastMath.floor(FastMath.abs(angle * INV_PITIMES2));
            newAngle = angle - times * PITIMES2;
        } else if (angle < 0) {
            times = (float) FastMath.ceil(FastMath.abs(angle * INV_PITIMES2));
            newAngle = angle + times * PITIMES2;
        } else {
            newAngle = angle;
        }

        return newAngle;
    }

}

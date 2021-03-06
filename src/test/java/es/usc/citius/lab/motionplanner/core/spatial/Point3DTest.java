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

import static es.usc.citius.lab.motionplanner.core.spatial.Point2DTest.random;

import es.usc.citius.lab.motionplanner.core.RepeatRule;
import es.usc.citius.lab.motionplanner.core.util.RandomUtils;
import org.apache.commons.math3.util.FastMath;
import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import static org.junit.Assert.*;
import org.junit.Before;

import static es.usc.citius.lab.motionplanner.core.spatial.Point3D.PRECISION;

/**
 * Tests for the class {@link Point3D}.
 *
 * @author Adrián González Sieira <<a href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>>
 * @since 01/09/2014
 */
public class Point3DTest{

    private float x1, y1, z1, x2, y2, z2;
    private Point3D point1, point2;
    private static float ERR = 1/PRECISION;
    private static float MAX = 20f;

    protected static final int EXECUTIONS = 10000;
    
    @Before
    public void setUp(){
        //generate x, y coordinates randomly above the compare precision treshold
        x1 = RandomUtils.randomToValue(random.nextFloat(), MAX);
        y1 = RandomUtils.randomToValue(random.nextFloat(), MAX);
        z1 = RandomUtils.randomToValue(random.nextFloat(), MAX);
        do {
            x2 = RandomUtils.randomToValue(random.nextFloat(), MAX);
        } while (Math.abs(x2 - x1) < 1E-2);
        do{
            y2 = RandomUtils.randomToValue(random.nextFloat(), MAX);
        } while (Math.abs(y2 - y1) < 1E-2);
        do{
            z2 = RandomUtils.randomToValue(random.nextFloat(), MAX);
        } while (Math.abs(z2 - z1) < 1E-2);

        //create new instance of point1
        point1 = new Point3D(x1, y1, z1);
        point2 = new Point3D(x2, y2, z2);
    }


    /**
     * Checks if the distance calculated between points is calculated properly.
     */
    @Test
    @RepeatRule.Repeat( times = EXECUTIONS)
    public void test_distance(){
        //distance operation
        float distance = point1.distance(point2);
        //obtain expected
        float expected = (float) FastMath.sqrt(FastMath.pow(x1 - x2, 2) + FastMath.pow(y1 - y2, 2) + FastMath.pow(z1 - z2, 2));
        assertEquals("[distance] wrong value", expected, distance, ERR);
    }

    /**
     * Checks if the adding operation is calculated properly.
     */
    @Test
    @RepeatRule.Repeat( times = EXECUTIONS)
    public void test_add(){
        //calculate correct values
        float x = x1 + x2;
        float y = y1 + y2;
        float z = z1 + z2;
        //sum operation
        point1.staticAdd(point2);
        //checking
        assertEquals("[add] wrong x value", x, point1.x, ERR);
        assertEquals("[add] wrong x value", y, point1.y, ERR);
        assertEquals("[add] wrong z value", z, point1.z, ERR);
    }

    /**
     * Checks if the adding operation is calculated properly.
     */
    @Test
    @RepeatRule.Repeat( times = EXECUTIONS)
    public void test_subtract(){
        //calculate correct values
        float x = x1 - x2;
        float y = y1 - y2;
        float z = z1 - z2;
        //sum operation
        point1.staticSubtract(point2);
        //checking
        assertEquals("[add] wrong x value", x, point1.x, ERR);
        assertEquals("[add] wrong x value", y, point1.y, ERR);
        assertEquals("[add] wrong z value", z, point1.z, ERR);
    }

    /**
     * Checks if the constructor generates the instances with the right content.
     */
    @Test
    @RepeatRule.Repeat( times = EXECUTIONS)
    public void test_creation() {
        assertEquals("[create] wrong x value", x1, point1.x, ERR);
        assertEquals("[create] wrong y value", y1, point1.y, ERR);
        assertEquals("[create] wrong z value", z1, point1.z, ERR);
        //obtain new instance from the last created
        Point3D test = new Point3D(point1);
        assertEquals("[clone] wrong x value", x1, test.x, ERR);
        assertEquals("[clone] wrong y value", y1, test.y, ERR);
        assertEquals("[clone] wrong y value", z1, test.z, ERR);
    }

    /**
     * Checks the size and content of the matrix generated by the {@link Point3D}.
     */
    @Test
    @RepeatRule.Repeat( times = EXECUTIONS)
    public void test_getMatrix(){
        //get matrix
        DenseMatrix64F matrix = point1.getMatrix();
        //check size
        assertTrue("[getMatrix] size is not 1x3", matrix.numRows == 3 && matrix.numCols == 1);
        //check elements
        assertEquals("[getMatrix] wrong x value", x1, matrix.get(0, 0), ERR);
        assertEquals("[getMatrix] wrong y value", y1, matrix.get(1, 0), ERR);
        assertEquals("[getMatrix] wrong y value", z1, matrix.get(2, 0), ERR);
    }

    /**
     * Checks if equal instances have equal hashcodes, and if different instances have
     * different ones.
     */
    @Test
    @RepeatRule.Repeat( times = EXECUTIONS)
    public void test_hashCode(){
        Point3D test = new Point3D(point1);
        Point3D test2 = new Point3D(point1.y, point1.x, point1.z);
        Point3D test3 = new Point3D(point1.x, point1.z, point1.y);
        assertEquals("[hashCode] wrong hashCode for equal instances", point1.hashCode(), test.hashCode());
        assertTrue("[hashCode] wrong hashCode", (point1.hashCode() == test2.hashCode()) == (Math.round(point1.x * PRECISION) == Math.round(test2.x * PRECISION) && Math.round(point1.y * PRECISION) == Math.round(test2.y * PRECISION) && Math.round(point1.z * PRECISION) == Math.round(test2.z * PRECISION)));
        assertTrue("[hashCode] wrong hashCode", (point1.hashCode() == test3.hashCode()) == (Math.round(point1.x * PRECISION) == Math.round(test3.x * PRECISION) && Math.round(point1.y * PRECISION) == Math.round(test3.y * PRECISION) && Math.round(point1.z * PRECISION) == Math.round(test3.z * PRECISION)));
    }

    /**
     * Checks if equal instances are detected as equal elements, and different
     * ones not.
     */
    @Test
    @RepeatRule.Repeat( times = EXECUTIONS)
    public void test_equals(){
        Point3D test = new Point3D(point1);
        Point3D test2 = new Point3D(point1.y, point1.x, point1.z);
        Point3D test3 = new Point3D(point1.x, point1.z, point1.y);
        assertEquals("[equals] wrong equals result for equal instances", point1, test);
        assertTrue("[equals] wrong equals result", point1.equals(test2) == (Math.round(point1.x * PRECISION) == Math.round(test2.x * PRECISION) && Math.round(point1.y * PRECISION) == Math.round(test2.y * PRECISION) && Math.round(point1.z * PRECISION) == Math.round(test2.z * PRECISION)));
        assertTrue("[equals] wrong equals result", point1.equals(test3) == (Math.round(point1.x * PRECISION) == Math.round(test3.x * PRECISION) && Math.round(point1.y * PRECISION) == Math.round(test3.y * PRECISION) && Math.round(point1.z * PRECISION) == Math.round(test3.z * PRECISION)));
    }

    
    @Test
    public void test01_rotateStaticYaw(){
        Point3D base1 = new Point3D(10, 0, 0);
        Point3D base2 = new Point3D(0, 10, 0);
        Point3D base3 = new Point3D(0, 0, 10);
        //rotate points in yaw
        Point3D point1 = base1.rotate((float) Math.PI/2, 0, 0);
        Point3D point2 = base2.rotate((float) Math.PI/2, 0, 0);
        Point3D point3 = base3.rotate((float) Math.PI/2, 0, 0);
        //test results for point 1
        assertEquals(0, point1.getX(), ERR);
        assertEquals(10, point1.getY(), ERR);
        assertEquals(0, point1.getZ(), ERR);
        //test results for point 2
        assertEquals(-10, point2.getX(), ERR);
        assertEquals(0, point2.getY(), ERR);
        assertEquals(0, point2.getZ(), ERR);
        //test results for point 3
        assertEquals(0, point3.getX(), ERR);
        assertEquals(0, point3.getY(), ERR);
        assertEquals(10, point3.getZ(), ERR);
        
        //rotate points in pitch
        point1 = base1.rotate(0, (float) Math.PI/2, 0);
        point2 = base2.rotate(0, (float) Math.PI/2, 0);
        point3 = base3.rotate(0, (float) Math.PI/2, 0);
        //test results for point 1
        assertEquals(0, point1.getX(), ERR);
        assertEquals(0, point1.getY(), ERR);
        assertEquals(-10, point1.getZ(), ERR);
        //test results for point 2
        assertEquals(0, point2.getX(), ERR);
        assertEquals(10, point2.getY(), ERR);
        assertEquals(0, point2.getZ(), ERR);
        //test results for point 3
        assertEquals(10, point3.getX(), ERR);
        assertEquals(0, point3.getY(), ERR);
        assertEquals(0, point3.getZ(), ERR);
        
        //rotate points in roll
        point1 = base1.rotate(0, 0, (float) Math.PI/2);
        point2 = base2.rotate( 0, 0, (float) Math.PI/2);
        point3 = base3.rotate( 0, 0, (float) Math.PI/2);
        //test results for point 1
        assertEquals(10, point1.getX(), ERR);
        assertEquals(0, point1.getY(), ERR);
        assertEquals(0, point1.getZ(), ERR);
        //test results for point 2
        assertEquals(0, point2.getX(), ERR);
        assertEquals(0, point2.getY(), ERR);
        assertEquals(10, point2.getZ(), ERR);
        //test results for point 3
        assertEquals(0, point3.getX(), ERR);
        assertEquals(-10, point3.getY(), ERR);
        assertEquals(0, point3.getZ(), ERR);
        
        //rotate points in yaw and pitch
        point1 = base1.rotate((float) Math.PI/4, (float) Math.PI/4, 0);
        point2 = base2.rotate((float) Math.PI/4, (float) Math.PI/4, 0);
        point3 = base3.rotate( (float) Math.PI/4, (float) Math.PI/4, 0);
        //see if we obtain the same results chaining several rotation operations
        Point3D point1Decoupled = base1.rotate(0, (float) Math.PI/4, 0).rotate((float) Math.PI/4, 0, 0);
        Point3D point2Decoupled = base2.rotate(0, (float) Math.PI/4, 0).rotate((float) Math.PI/4, 0, 0);
        Point3D point3Decoupled = base3.rotate(0, (float) Math.PI/4, 0).rotate((float) Math.PI/4, 0, 0);
        //test results for point 1
        assertEquals(point1.x, point1Decoupled.x, ERR);
        assertEquals(point1.y, point1Decoupled.y, ERR);
        assertEquals(point1.z, point1Decoupled.z, ERR);
        //test results for point 2
        assertEquals(point2.x, point2Decoupled.x, ERR);
        assertEquals(point2.y, point2Decoupled.y, ERR);
        assertEquals(point2.z, point2Decoupled.z, ERR);
        //test results for point 3
        assertEquals(point3.x, point3Decoupled.x, ERR);
        assertEquals(point3.y, point3Decoupled.y, ERR);
        assertEquals(point3.z, point3Decoupled.z, ERR);
        
        //rotate points in yaw and pitch and roll
        point1 = base1.rotate( (float) Math.PI/4, (float) Math.PI/4, (float) Math.PI/4);
        point2 = base2.rotate( (float) Math.PI/4, (float) Math.PI/4, (float) Math.PI/4);
        point3 = base3.rotate( (float) Math.PI/4, (float) Math.PI/4, (float) Math.PI/4);
        //see if we obtain the same results chaining several rotation operations
        point1Decoupled = base1.rotate( 0, 0, (float) Math.PI/4).rotate(0, (float) Math.PI/4, 0).rotate((float) Math.PI/4, 0, 0);
        point2Decoupled = base2.rotate( 0, 0, (float) Math.PI/4).rotate(0, (float) Math.PI/4, 0).rotate((float) Math.PI/4, 0, 0);
        point3Decoupled = base3.rotate( 0, 0, (float) Math.PI/4).rotate(0, (float) Math.PI/4, 0).rotate((float) Math.PI/4, 0, 0);
        //test results for point 1
        assertEquals(point1.x, point1Decoupled.x, ERR);
        assertEquals(point1.y, point1Decoupled.y, ERR);
        assertEquals(point1.z, point1Decoupled.z, ERR);
        //test results for point 2
        assertEquals(point2.x, point2Decoupled.x, ERR);
        assertEquals(point2.y, point2Decoupled.y, ERR);
        assertEquals(point2.z, point2Decoupled.z, ERR);
        //test results for point 3
        assertEquals(point3.x, point3Decoupled.x, ERR);
        assertEquals(point3.y, point3Decoupled.y, ERR);
        assertEquals(point3.z, point3Decoupled.z, ERR);
    }
}

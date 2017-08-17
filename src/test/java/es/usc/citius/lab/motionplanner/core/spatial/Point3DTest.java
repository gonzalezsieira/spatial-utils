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

import es.usc.citius.lab.motionplanner.core.util.RandomUtils;
import org.apache.commons.math3.util.FastMath;
import org.junit.Test;
import static org.junit.Assert.*;
import org.junit.Before;

/**
 * Tests for the class {@link Point3D}.
 *
 * @author Adrián González Sieira <<a href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>>
 * @since 01/09/2014
 */
public class Point3DTest{
    
    private float yaw, pitch, roll;
    private Point3D point;
    private static float ERR = (float) 1E-3;
    private static float MAX = 20f;
    
    @Before
    public void setUp(){
        float x, y, z;
        //angles generated randomly above the precision compare treshold
        yaw = RandomUtils.randomToValue(random.nextFloat(), (float) Math.PI);
        pitch = RandomUtils.randomToValue(random.nextFloat(), (float) Math.PI);
        roll = RandomUtils.randomToValue(random.nextFloat(), (float) Math.PI);
        //point generated randomly
        x = RandomUtils.randomToValue(random.nextFloat(), MAX);
        y = RandomUtils.randomToValue(random.nextFloat(), MAX);
        z = RandomUtils.randomToValue(random.nextFloat(), MAX);
        //instantiate point
        point = new Point3D(x, y, z);
    }
    
    
    @Test
    public void test01_rotateStaticYaw(){
        Point3D base1 = new Point3D(10, 0, 0);
        Point3D base2 = new Point3D(0, 10, 0);
        Point3D base3 = new Point3D(0, 0, 10);
        //rotate points in yaw
        Point3D point1 = Point3D.rotate(base1, (float) Math.PI/2, 0, 0);
        Point3D point2 = Point3D.rotate(base2, (float) Math.PI/2, 0, 0);
        Point3D point3 = Point3D.rotate(base3, (float) Math.PI/2, 0, 0);
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
        point1 = Point3D.rotate(base1, 0, (float) Math.PI/2, 0);
        point2 = Point3D.rotate(base2, 0, (float) Math.PI/2, 0);
        point3 = Point3D.rotate(base3, 0, (float) Math.PI/2, 0);
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
        point1 = Point3D.rotate(base1, 0, 0, (float) Math.PI/2);
        point2 = Point3D.rotate(base2, 0, 0, (float) Math.PI/2);
        point3 = Point3D.rotate(base3, 0, 0, (float) Math.PI/2);
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
        point1 = Point3D.rotate(base1, (float) Math.PI/4, (float) Math.PI/4, 0);
        point2 = Point3D.rotate(base2, (float) Math.PI/4, (float) Math.PI/4, 0);
        point3 = Point3D.rotate(base3, (float) Math.PI/4, (float) Math.PI/4, 0);
        //see if we obtain the same results chaining several rotation operations
        Point3D point1Decoupled = Point3D.rotate(Point3D.rotate(base1, 0, (float) Math.PI/4, 0), (float) Math.PI/4, 0, 0);
        Point3D point2Decoupled = Point3D.rotate(Point3D.rotate(base2, 0, (float) Math.PI/4, 0), (float) Math.PI/4, 0, 0);
        Point3D point3Decoupled = Point3D.rotate(Point3D.rotate(base3, 0, (float) Math.PI/4, 0), (float) Math.PI/4, 0, 0);
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
        point1 = Point3D.rotate(base1, (float) Math.PI/4, (float) Math.PI/4, (float) Math.PI/4);
        point2 = Point3D.rotate(base2, (float) Math.PI/4, (float) Math.PI/4, (float) Math.PI/4);
        point3 = Point3D.rotate(base3, (float) Math.PI/4, (float) Math.PI/4, (float) Math.PI/4);
        //see if we obtain the same results chaining several rotation operations
        point1Decoupled = Point3D.rotate(Point3D.rotate(Point3D.rotate(base1, 0, 0, (float) Math.PI/4), 0, (float) Math.PI/4, 0), (float) Math.PI/4, 0, 0);
        point2Decoupled = Point3D.rotate(Point3D.rotate(Point3D.rotate(base2, 0, 0, (float) Math.PI/4), 0, (float) Math.PI/4, 0), (float) Math.PI/4, 0, 0);
        point3Decoupled = Point3D.rotate(Point3D.rotate(Point3D.rotate(base3, 0, 0, (float) Math.PI/4), 0, (float) Math.PI/4, 0), (float) Math.PI/4, 0, 0);
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

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
package es.usc.citius.lab.motionplanner.core.shapes;

import es.usc.citius.lab.motionplanner.core.RepeatRule;
import es.usc.citius.lab.motionplanner.core.spatial.Point2D;
import es.usc.citius.lab.motionplanner.core.spatial.Point3D;
import es.usc.citius.lab.motionplanner.core.spatial.Pose2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.apache.commons.math3.util.FastMath;
import org.junit.Test;
import static org.junit.Assert.*;
import org.junit.Before;

/**
 * Test methods for class {@link Shape2D}.
 *
 * @author Adrián González Sieira <a href=mailto:adrian.gonzalez@usc.es>adrian.gonzalez@usc.es</a>
 * @since 04/09/2014
 */
public class ShapeSquare2DTest {
    
    private static final float SHAPE_DIM_MAX = 100;
    private static final double ERR = 1E-4;
    private static final Random RANDOM = new Random(System.currentTimeMillis());
    private float dx, dy;
    private ShapeSquare2D shape;
    
    @Before
    public void setUp(){
        dx = SHAPE_DIM_MAX * RANDOM.nextFloat();
        dy = SHAPE_DIM_MAX * RANDOM.nextFloat();
        shape = new ShapeSquare2D(
                dx, 
                dy
        );
    }
    
    /**
     * Tests that the corners are generated properly.
     */
    @RepeatRule.Repeat(times = 100)
    @Test
    public void test_corners(){
        //generate the values of the angles of the four corners
        List<Double> angles = new ArrayList<Double>();
        angles.add(FastMath.atan2(dy, dx));
        angles.add(FastMath.atan2(dy, -dx));
        angles.add(FastMath.atan2(-dy, dx));
        angles.add(FastMath.atan2(-dy, -dx));
        //generate the distance between the center and the corners
        double distance = FastMath.hypot(dx / 2, dy / 2);
        //check the corners are generated properly (in angle and distance)
        for(Point3D current : shape.vertexAt(Pose2D.ZERO)){
            //test distance
            assertEquals(current.distance(Point3D.ZERO), distance, ERR);
            //test angle
            //angle of the current corner
            double currentAngle = Point3D.ZERO.yawTo(current);
            boolean found = false;
            //check if the angle is in the list of angles of the corners
            for(double listAngle : angles){
                if(FastMath.abs(currentAngle - listAngle) < ERR){
                    found = true;
                    break;
                }
            }
            assertTrue("current angle " + currentAngle + " does not match with a corner", found);
        }
    }
    
    @Test
    public void test_distanceVectorToPoint(){
        ShapeSquare2D shapeSquared = new ShapeSquare2D(2, 2);
        //test diagonals
        double[][] d1 = shapeSquared.distanceVectorToPoint(Pose2D.ZERO, new Point2D(3, 3));
        double[][] d2 = shapeSquared.distanceVectorToPoint(Pose2D.ZERO, new Point2D(3, -3));
        double[][] d3 = shapeSquared.distanceVectorToPoint(Pose2D.ZERO, new Point2D(-3, 3));
        double[][] d4 = shapeSquared.distanceVectorToPoint(Pose2D.ZERO, new Point2D(-3, -3));
        assertArrayEquals("45 degree point", new double[][]{{2}, {2}}, d1);
        assertArrayEquals("-45 degree point", new double[][]{{2}, {-2}}, d2);
        assertArrayEquals("135 degree point", new double[][]{{-2}, {2}}, d3);
        assertArrayEquals("-135 degree point", new double[][]{{-2}, {-2}}, d4);
        //test points near to corners
        double[][] d5_1 = shapeSquared.distanceVectorToPoint(Pose2D.ZERO, new Point2D(1, 3));
        double[][] d5_2 = shapeSquared.distanceVectorToPoint(Pose2D.ZERO, new Point2D(3, 1));
        double[][] d6_1 = shapeSquared.distanceVectorToPoint(Pose2D.ZERO, new Point2D(1, -3));
        double[][] d6_2 = shapeSquared.distanceVectorToPoint(Pose2D.ZERO, new Point2D(3, -1));
        double[][] d7_1 = shapeSquared.distanceVectorToPoint(Pose2D.ZERO, new Point2D(-1, 3));
        double[][] d7_2 = shapeSquared.distanceVectorToPoint(Pose2D.ZERO, new Point2D(-3, 1));
        double[][] d8_1 = shapeSquared.distanceVectorToPoint(Pose2D.ZERO, new Point2D(-1, -3));
        double[][] d8_2 = shapeSquared.distanceVectorToPoint(Pose2D.ZERO, new Point2D(-3, -1));
        assertArrayEquals("45 degree corner: vertical", new double[][]{{0}, {2}}, d5_1);
        assertArrayEquals("45 degree corner: horizontal", new double[][]{{2}, {0}}, d5_2);
        assertArrayEquals("-45 degree corner: vertical", new double[][]{{0}, {-2}}, d6_1);
        assertArrayEquals("-45 degree corner: horizontal", new double[][]{{2}, {0}}, d6_2);
        assertArrayEquals("135 degree point", new double[][]{{0}, {2}}, d7_1);
        assertArrayEquals("-135 degree point", new double[][]{{-2}, {0}}, d7_2);
        assertArrayEquals("-135 degree point", new double[][]{{0}, {-2}}, d8_1);
        assertArrayEquals("-135 degree point", new double[][]{{-2}, {0}}, d8_2);
    }
    
}

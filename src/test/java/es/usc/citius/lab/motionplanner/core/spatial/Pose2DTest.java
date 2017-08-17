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

import es.usc.citius.lab.motionplanner.core.RepeatRule.Repeat;
import static es.usc.citius.lab.motionplanner.core.spatial.Point2DTest.EXECUTIONS;
import static es.usc.citius.lab.motionplanner.core.spatial.Point2DTest.random;
import es.usc.citius.lab.motionplanner.core.util.MathFunctions;
import es.usc.citius.lab.motionplanner.core.util.RandomUtils;
import org.apache.commons.math3.util.FastMath;
import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;
import org.junit.Test;

/**
 * * Test case for the functionalities of {@link Pose2D}.
 * 
 * @author Adrián González Sieira <a href=mailto:adrian.gonzalez@usc.es>adrian.gonzalez@usc.es</a>
 */
public class Pose2DTest extends Point2DTest{
    
    protected float yaw1, yaw2;
    protected Pose2D pose1;
    protected Pose2D pose2;
    
    @Override
    public void setUp() {
        super.setUp();
        //generate angles randomly above the precision compare treshold
        yaw1 = RandomUtils.randomToValue(random.nextFloat(), (float) Math.PI);
        yaw2 = RandomUtils.randomToValue(random.nextFloat(), (float) Math.PI);
        //create new instance of point1
        pose1 = new Pose2D(x1, y1, yaw1);
        pose2 = new Pose2D(x2, y2, yaw2);
    }

    @Override
    public void test_creation() {
        assertEquals("[create] wrong x value", x1, pose1.x, ERR);
        assertEquals("[create] wrong y value", y1, pose1.y, ERR);
        assertEquals("[create] wrong yaw value", yaw1, pose1.yaw, ERR);
        //obtain new instance from the last created
        Pose2D test = new Pose2D(pose1);
        assertEquals("[clone] wrong x value", x1, test.x, ERR);
        assertEquals("[clone] wrong y value", y1, test.y, ERR);
        assertEquals("[clone] wrong yaw value", yaw1, test.yaw, ERR);
        //obtain new instance from the point created and the yaw
        test = new Pose2D(point1, yaw1);
        assertEquals("[clone] wrong x value", x1, test.x, ERR);
        assertEquals("[clone] wrong y value", y1, test.y, ERR);
        assertEquals("[clone] wrong yaw value", yaw1, test.yaw, ERR);
    }

    @Override
    public void test_hashCode() {
        Pose2D test = new Pose2D(pose1);
        Pose2D test2 = new Pose2D(pose1.y, pose1.x, pose1.yaw);
        Pose2D test3 = new Pose2D(pose1.x, pose1.y, pose2.yaw);
        assertEquals("[hashCode] wrong hashCode for equal instances", pose1.hashCode(), test.hashCode());
        assertTrue("[hashCode] wrong hashCode", (pose1.hashCode() == test2.hashCode()) == (Math.abs(pose1.x - test2.x) < 1E-4 && Math.abs(pose1.y - test2.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(pose1.yaw, test2.yaw)) < 1E-4));
        assertTrue("[hashCode] wrong hashCode", (pose1.hashCode() == test3.hashCode()) == (Math.abs(pose1.x - test3.x) < 1E-4 && Math.abs(pose1.y - test3.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(pose1.yaw, test3.yaw)) < 1E-4));
    }

    @Override
    public void test_equals() {
        Pose2D test = new Pose2D(pose1);
        Pose2D test2 = new Pose2D(pose1.y, pose1.x, pose1.yaw);
        Pose2D test3 = new Pose2D(pose1.x, pose1.y, pose2.yaw);
        assertEquals("[hashCode] wrong equals for equal instances", pose1, test);
        assertTrue("[equals] wrong result", (pose1.equals(test2)) == (Math.abs(pose1.x - test2.x) < 1E-4 && Math.abs(pose1.y - test2.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(pose1.yaw, test2.yaw)) < 1E-4));
        assertTrue("[equals] wrong result", (pose1.equals(test3)) == (Math.abs(pose1.x - test3.x) < 1E-4 && Math.abs(pose1.y - test3.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(pose1.yaw, test3.yaw)) < 1E-4));
    }

    @Override
    public void test_getMatrix(){
        //get matrix
        DenseMatrix64F matrix = pose1.getMatrix();
        //check size
        assertTrue("[getMatrix] size is not 1x3", matrix.numRows == 3 && matrix.numCols == 1);
        //check elements
        assertEquals("[getMatrix] wrong x value", x1, matrix.get(0, 0), ERR);
        assertEquals("[getMatrix] wrong y value", y1, matrix.get(1, 0), ERR);
        assertEquals("[getMatrix] wrong yaw value", yaw1, matrix.get(2, 0), ERR);
    }
    
    /**
     * Checks if the relative angle between a position and a point taking into
     * account the heading of the pose is calculated properly.
     */
    @Test
    @Repeat( times = EXECUTIONS)
    public void test_relativeYawTo(){
        //atan2(point2 - pose1) - pose1.yaw
        float result = pose1.relativeYawTo(point2);
        //expected result
        float expected = MathFunctions.adjustAngleP((float) FastMath.atan2(y2 - y1, x2 - x1) - yaw1);
        assertEquals("[absoluteAngle1P] wrong x value", expected, result, ERR);

    }

    @Override
    public void test_rotate() {
        //random angle
        float angle = (random.nextFloat() - 0.5f) * 2 * (float)FastMath.PI;
        //calculate correct values
        SimpleMatrix expected = new SimpleMatrix(new double[][]{{FastMath.cos(angle), -FastMath.sin(angle), 0}, {FastMath.sin(angle), FastMath.cos(angle), 0}, {0, 0, (angle + yaw1)/yaw1}}).mult(new SimpleMatrix(pose1.getMatrix()));
        //rotate operation
        pose1.rotate(angle);
        //checking
        assertEquals("[rotate] wrong x value", expected.get(0), pose1.x, 1E-3);
        assertEquals("[rotate] wrong y value", expected.get(1), pose1.y, 1E-3);
        assertEquals("[static rotate] wrong yaw value", MathFunctions.adjustAngleP((float) expected.get(2)), pose1.yaw, 1E-3);
    }
  
    @Override
    public void test_staticAdd() {
        //calculate correct values
        float x = x1 + x2;
        float y = y1 + y2;
        //static sum operation
        Pose2D result = Pose2D.add(pose1, point2);
        //checking
        assertEquals("[static add] wrong x value", x, result.x, ERR);
        assertEquals("[static add] wrong x value", y, result.y, ERR);
        assertEquals("[static add] wrong yaw value", yaw1, result.yaw, ERR);
    }

    @Override
    public void test_staticSubtract() {
        //calculate correct values
        float x = x1 - x2;
        float y = y1 - y2;
        //static subtract operation
        Pose2D result = Pose2D.subtract(pose1, point2);
        //checking
        assertEquals("[static subtract] wrong x value", x, result.x, ERR);
        assertEquals("[static subtract] wrong y value", y, result.y, ERR);
        assertEquals("[static subtract] wrong yaw value", yaw1, result.yaw, ERR);
    }
    
    @Override
    public void test_staticRotate() {
        //random angle
        float angle = (random.nextFloat() - 0.5f) * 2 * (float)FastMath.PI;
        //calculate correct values
        SimpleMatrix expected = new SimpleMatrix(new double[][]{{FastMath.cos(angle), -FastMath.sin(angle), 0}, {FastMath.sin(angle), FastMath.cos(angle), 0}, {0, 0, (angle + yaw1)/yaw1}}).mult(new SimpleMatrix(pose1.getMatrix()));
        //rotate operation
        Pose2D result = Pose2D.rotate(pose1, angle);
        //checking
        assertEquals("[static rotate] wrong x value", expected.get(0), result.x, 1E-3);
        assertEquals("[static rotate] wrong y value", expected.get(1), result.y, 1E-3);
        assertEquals("[static rotate] wrong yaw value", MathFunctions.adjustAngleP((float) expected.get(2)), result.yaw, 1E-3);
    }
    
    /**
     * Checks the symmetry calculation respect to axis X.
     */
    @Test
    @Repeat( times = EXECUTIONS)
    public void test_symmetricAxisX(){
        Pose2D result = pose1.symmetricAxisX();
        assertEquals("[symmetricAxisX] wrong x value", x1, result.x, ERR);
        assertEquals("[symmetricAxisX] wrong y value", -y1, result.y, ERR);
        assertEquals("[symmetricAxisX] wrong yaw value", -yaw1, result.yaw, ERR);
    }
    
    
    /**
     * Checks the symmetry calculation respect to axis Y.
     */
    @Test
    @Repeat( times = EXECUTIONS)
    public void test_symmetricAxisY(){
        Pose2D result = pose1.symmetricAxisY();
        assertEquals("[symmetricAxisX] wrong x value", -x1, result.x, ERR);
        assertEquals("[symmetricAxisX] wrong y value", y1, result.y, ERR);
        assertEquals("[symmetricAxisX] wrong yaw value", MathFunctions.adjustAngleP(MathFunctions.PI - yaw1), result.yaw, ERR);
    }
}

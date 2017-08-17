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
import es.usc.citius.lab.motionplanner.core.util.MathFunctions;
import es.usc.citius.lab.motionplanner.core.util.RandomUtils;
import org.apache.commons.math3.util.FastMath;
import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;
import static org.junit.Assert.*;
import org.junit.Test;

/**
 * * Test case for the functionalities of {@link State2D}.
 * 
 * @author Adrián González Sieira <a href=mailto:adrian.gonzalez@usc.es>adrian.gonzalez@usc.es</a>
 */
public class State2DTest extends Pose2DTest{
    
    protected float vx1, vy1, w1, vx2, vy2, w2;
    
    protected State2D state1, state2;

    @Override
    public void setUp() {
        super.setUp();
        super.setUp();
        //generate vx, w values randomly, above the precision compare treshold
        vx1 = RandomUtils.randomToValue(random.nextFloat(), 0.5f);
        vy1 = RandomUtils.randomToValue(random.nextFloat(), 0.5f);
        w1 = RandomUtils.randomToValue(random.nextFloat(), (float) Math.PI);
        vx2 = RandomUtils.randomToValue(random.nextFloat(), 0.5f);
        vy2 = RandomUtils.randomToValue(random.nextFloat(), 0.5f);
        w2 = RandomUtils.randomToValue(random.nextFloat(), (float) Math.PI);
        //create new instance of state1/state2
        state1 = new State2D(x1, y1, yaw1, vx1, vy1, w1);
        state2 = new State2D(x2, y2, yaw2, vx2, vy2, w2);
    }

    @Override
    public void test_creation() {
        assertEquals("[create] wrong x value", x1, state1.x, ERR);
        assertEquals("[create] wrong y value", y1, state1.y, ERR);
        assertEquals("[create] wrong yaw value", yaw1, state1.yaw, ERR);
        assertEquals("[create] wrong vx value", vx1, state1.vx, ERR);
        assertEquals("[create] wrong vy value", vy1, state1.vy, ERR);
        assertEquals("[create] wrong w value", w1, state1.w, ERR);
        //obtain new instance using the clone constructor
        State2D test = new State2D(state1);
        assertEquals("[clone] wrong x value", x1, test.x, ERR);
        assertEquals("[clone] wrong y value", y1, test.y, ERR);
        assertEquals("[clone] wrong yaw value", yaw1, test.yaw, ERR);
        assertEquals("[clone] wrong vx value", vx1, test.vx, ERR);
        assertEquals("[clone] wrong vy value", vy1, test.vy, ERR);
        assertEquals("[clone] wrong w value", w1, test.w, ERR);
        //obtain new instance using the matrix constructor
        test = new State2D(state1.getMatrix());
        assertEquals("[clone] wrong x value", x1, test.x, ERR);
        assertEquals("[clone] wrong y value", y1, test.y, ERR);
        assertEquals("[clone] wrong yaw value", yaw1, test.yaw, ERR);
        assertEquals("[clone] wrong vx value", vx1, test.vx, ERR);
        assertEquals("[clone] wrong vy value", vy1, test.vy, ERR);
        assertEquals("[clone] wrong w value", w1, test.w, ERR);
        //obtain new instance from the pose created and the yaw
        test = new State2D(pose1, vx1, vy1, w1);
        assertEquals("[create] wrong x value", x1, test.x, ERR);
        assertEquals("[create] wrong y value", y1, test.y, ERR);
        assertEquals("[create] wrong yaw value", yaw1, test.yaw, ERR);
        assertEquals("[create] wrong vx value", vx1, test.vx, ERR);
        assertEquals("[create] wrong vy value", vy1, test.vy, ERR);
        assertEquals("[create] wrong w value", w1, test.w, ERR);
    }

    @Override
    public void test_hashCode() {
        State2D test = new State2D(state1);
        State2D test2 = new State2D(state1.y, state1.x, state1.yaw, state1.vx, state1.vy, state1.w);
        State2D test3 = new State2D(-state1.x, state1.y, state1.yaw, state1.vx, state1.vy, state1.w);
        State2D test4 = new State2D(state1.x, -state1.y, state1.yaw, state1.vx, state1.vy, state1.w);
        State2D test5 = new State2D(state1.x, state1.y, -state1.yaw, state1.vx, state1.vy, state1.w);
        State2D test6 = new State2D(state1.x, state1.y, state1.yaw, -state1.vx, state1.vy, state1.w);
        State2D test7 = new State2D(state1.x, state1.y, state1.yaw, state1.vx, -state1.vy, state1.w);
        State2D test8 = new State2D(state1.x, state1.y, state1.yaw, state1.vx, state1.vy, -state1.w);
        assertEquals("[hashCode] wrong hashCode for equal instances", state1.hashCode(), test.hashCode());
        assertTrue("[hashCode] wrong hashCode", (test.hashCode() == test2.hashCode()) == (Math.abs(test.x - test2.x) < 1E-4 && Math.abs(test.y - test2.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(test.yaw, test2.yaw)) < 1E-4 && Math.abs(test.vx - test2.vx) < 1E-4 && Math.abs(test.vy - test2.vy) < 1E-4 && Math.abs(test.w - test2.w) < 1E-4));
        assertTrue("[hashCode] wrong hashCode", (test.hashCode() == test3.hashCode()) == (Math.abs(test.x - test3.x) < 1E-4 && Math.abs(test.y - test3.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(test.yaw, test3.yaw)) < 1E-4 && Math.abs(test.vx - test3.vx) < 1E-4 && Math.abs(test.vy - test3.vy) < 1E-4 && Math.abs(test.w - test3.w) < 1E-4));
        assertTrue("[hashCode] wrong hashCode", (test.hashCode() == test4.hashCode()) == (Math.abs(test.x - test4.x) < 1E-4 && Math.abs(test.y - test4.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(test.yaw, test4.yaw)) < 1E-4 && Math.abs(test.vx - test4.vx) < 1E-4 && Math.abs(test.vy - test4.vy) < 1E-4 && Math.abs(test.w - test4.w) < 1E-4));
        assertTrue("[hashCode] wrong hashCode", (test.hashCode() == test5.hashCode()) == (Math.abs(test.x - test5.x) < 1E-4 && Math.abs(test.y - test5.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(test.yaw, test5.yaw)) < 1E-4 && Math.abs(test.vx - test5.vx) < 1E-4 && Math.abs(test.vy - test5.vy) < 1E-4 && Math.abs(test.w - test5.w) < 1E-4));
        assertTrue("[hashCode] wrong hashCode", (test.hashCode() == test6.hashCode()) == (Math.abs(test.x - test6.x) < 1E-4 && Math.abs(test.y - test6.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(test.yaw, test6.yaw)) < 1E-4 && Math.abs(test.vx - test6.vx) < 1E-4 && Math.abs(test.vy - test6.vy) < 1E-4 && Math.abs(test.w - test6.w) < 1E-4));
        assertTrue("[hashCode] wrong hashCode", (test.hashCode() == test7.hashCode()) == (Math.abs(test.x - test7.x) < 1E-4 && Math.abs(test.y - test7.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(test.yaw, test7.yaw)) < 1E-4 && Math.abs(test.vx - test7.vx) < 1E-4 && Math.abs(test.vy - test7.vy) < 1E-4 && Math.abs(test.w - test7.w) < 1E-4));
        assertTrue("[hashCode] wrong hashCode", (test.hashCode() == test8.hashCode()) == (Math.abs(test.x - test8.x) < 1E-4 && Math.abs(test.y - test8.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(test.yaw, test8.yaw)) < 1E-4 && Math.abs(test.vx - test8.vx) < 1E-4 && Math.abs(test.vy - test8.vy) < 1E-4 && Math.abs(test.w - test8.w) < 1E-4));
    }

    @Override
    public void test_equals() {
        State2D test = new State2D(state1);
        State2D test2 = new State2D(state1.y, state1.x, state1.yaw, state1.vx, state1.vy, state1.w);
        State2D test3 = new State2D(-state1.x, state1.y, state1.yaw, state1.vx, state1.vy, state1.w);
        State2D test4 = new State2D(state1.x, -state1.y, state1.yaw, state1.vx, state1.vy, state1.w);
        State2D test5 = new State2D(state1.x, state1.y, -state1.yaw, state1.vx, state1.vy, state1.w);
        State2D test6 = new State2D(state1.x, state1.y, state1.yaw, -state1.vx, state1.vy, state1.w);
        State2D test7 = new State2D(state1.x, state1.y, state1.yaw, state1.vx, -state1.vy, state1.w);
        State2D test8 = new State2D(state1.x, state1.y, state1.yaw, state1.vx, state1.vy, -state1.w);
        assertEquals("[equals] wrong equals for equal instances", state1, test);
        assertTrue("[equals] wrong hashCode", (test.equals(test2)) == (Math.abs(test.x - test2.x) < 1E-4 && Math.abs(test.y - test2.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(test.yaw, test2.yaw)) < 1E-4 && Math.abs(test.vx - test2.vx) < 1E-4 && Math.abs(test.vy - test2.vy) < 1E-4 && Math.abs(test.w - test2.w) < 1E-4));
        assertTrue("[equals] wrong hashCode", (test.equals(test3)) == (Math.abs(test.x - test3.x) < 1E-4 && Math.abs(test.y - test3.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(test.yaw, test3.yaw)) < 1E-4 && Math.abs(test.vx - test3.vx) < 1E-4 && Math.abs(test.vy - test3.vy) < 1E-4 && Math.abs(test.w - test3.w) < 1E-4));
        assertTrue("[equals] wrong hashCode", (test.equals(test4)) == (Math.abs(test.x - test4.x) < 1E-4 && Math.abs(test.y - test4.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(test.yaw, test4.yaw)) < 1E-4 && Math.abs(test.vx - test4.vx) < 1E-4 && Math.abs(test.vy - test4.vy) < 1E-4 && Math.abs(test.w - test4.w) < 1E-4));
        assertTrue("[equals] wrong hashCode", (test.equals(test5)) == (Math.abs(test.x - test5.x) < 1E-4 && Math.abs(test.y - test5.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(test.yaw, test5.yaw)) < 1E-4 && Math.abs(test.vx - test5.vx) < 1E-4 && Math.abs(test.vy - test5.vy) < 1E-4 && Math.abs(test.w - test5.w) < 1E-4));
        assertTrue("[equals] wrong hashCode", (test.equals(test6)) == (Math.abs(test.x - test6.x) < 1E-4 && Math.abs(test.y - test6.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(test.yaw, test6.yaw)) < 1E-4 && Math.abs(test.vx - test6.vx) < 1E-4 && Math.abs(test.vy - test6.vy) < 1E-4 && Math.abs(test.w - test6.w) < 1E-4));
        assertTrue("[equals] wrong hashCode", (test.equals(test7)) == (Math.abs(test.x - test7.x) < 1E-4 && Math.abs(test.y - test7.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(test.yaw, test7.yaw)) < 1E-4 && Math.abs(test.vx - test7.vx) < 1E-4 && Math.abs(test.vy - test7.vy) < 1E-4 && Math.abs(test.w - test7.w) < 1E-4));
        assertTrue("[equals] wrong hashCode", (test.equals(test8)) == (Math.abs(test.x - test8.x) < 1E-4 && Math.abs(test.y - test8.y) < 1E-4 && Math.abs(MathFunctions.errorBetweenAngles(test.yaw, test8.yaw)) < 1E-4 && Math.abs(test.vx - test8.vx) < 1E-4 && Math.abs(test.vy - test8.vy) < 1E-4 && Math.abs(test.w - test8.w) < 1E-4));
    }

    @Override
    public void test_getMatrix() {
        //get matrix
        DenseMatrix64F matrix = state1.getMatrix();
        //check size
        assertTrue("[getMatrix] size is not 6x1", matrix.numRows == 6 && matrix.numCols == 1);
        //check elements
        assertEquals("[getMatrix] wrong x value", x1, matrix.get(0, 0), ERR);
        assertEquals("[getMatrix] wrong y value", y1, matrix.get(1, 0), ERR);
        assertEquals("[getMatrix] wrong yaw value", yaw1, matrix.get(2, 0), ERR);
        assertEquals("[getMatrix] wrong vx value", vx1, matrix.get(3, 0), ERR);
        assertEquals("[getMatrix] wrong vy value", vy1, matrix.get(4, 0), ERR);
        assertEquals("[getMatrix] wrong w value", w1, matrix.get(5, 0), ERR);
    }
    
    /**
     * Checks the symmetry calculation respect to axis X.
     */
    @Test
    @Repeat( times = EXECUTIONS)
    @Override
    public void test_symmetricAxisX(){
        State2D result = state1.symmetricAxisX();
        assertEquals("[symmetricAxisX] wrong x value", x1, result.x, ERR);
        assertEquals("[symmetricAxisX] wrong y value", -y1, result.y, ERR);
        assertEquals("[symmetricAxisX] wrong yaw value", -yaw1, result.yaw, ERR);
        assertEquals("[symmetricAxisX] wrong vx value", vx1, result.vx, ERR);
        assertEquals("[symmetricAxisX] wrong vy value", -vy1, result.vy, ERR);
        assertEquals("[symmetricAxisX] wrong w value", -w1, result.w, ERR);
    }
    
    
    /**
     * Checks the symmetry calculation respect to axis Y.
     */
    @Test
    @Repeat( times = EXECUTIONS)
    @Override
    public void test_symmetricAxisY(){
        State2D result = state1.symmetricAxisY();
        assertEquals("[symmetricAxisX] wrong x value", -x1, result.x, ERR);
        assertEquals("[symmetricAxisX] wrong y value", y1, result.y, ERR);
        assertEquals("[symmetricAxisX] wrong yaw value", MathFunctions.adjustAngleP(MathFunctions.PI - yaw1), result.yaw, ERR);
        assertEquals("[symmetricAxisX] wrong vx value", vx1, result.vx, ERR);
        assertEquals("[symmetricAxisX] wrong vy value", -vy1, result.vy, ERR);
        assertEquals("[symmetricAxisX] wrong w value", -w1, result.w, ERR);
    }
    
    @Override
    public void test_staticAdd() {
        //calculate correct values
        float x = x1 + x2;
        float y = y1 + y2;
        //static sum operation
        State2D result = State2D.add(state1, point2);
        //checking
        assertEquals("[static add] wrong x value", x, result.x, ERR);
        assertEquals("[static add] wrong x value", y, result.y, ERR);
        assertEquals("[static add] wrong yaw value", yaw1, result.yaw, ERR);
        assertEquals("[static add] wrong vx value", vx1, result.vx, ERR);
        assertEquals("[static add] wrong vy value", vy1, result.vy, ERR);
        assertEquals("[static add] wrong w value", w1, result.w, ERR);
    }

    @Override
    public void test_staticSubtract() {
        //calculate correct values
        float x = x1 - x2;
        float y = y1 - y2;
        //static subtract operation
        State2D result = State2D.subtract(state1, point2);
        //checking
        assertEquals("[static subtract] wrong x value", x, result.x, ERR);
        assertEquals("[static subtract] wrong y value", y, result.y, ERR);
        assertEquals("[static subtract] wrong yaw value", yaw1, result.yaw, ERR);
        assertEquals("[static subtract] wrong vx value", vx1, result.vx, ERR);
        assertEquals("[static subtract] wrong vy value", vy1, result.vy, ERR);
        assertEquals("[static subtract] wrong w value", w1, result.w, ERR);
    }
    
    @Override
    public void test_staticRotate() {
        //random angle
        float angle = (random.nextFloat() - 0.5f) * 2 * (float)FastMath.PI;
        //calculate correct values
        SimpleMatrix expected = new SimpleMatrix(new double[][]{{FastMath.cos(angle), -FastMath.sin(angle), 0}, {FastMath.sin(angle), FastMath.cos(angle), 0}, {0, 0, (angle + yaw1)/yaw1}}).mult(new SimpleMatrix(pose1.getMatrix()));
        //rotate operation
        State2D result = State2D.rotate(state1, angle);
        //checking
        assertEquals("[static rotate] wrong x value", expected.get(0), result.x, 1E-3);
        assertEquals("[static rotate] wrong y value", expected.get(1), result.y, 1E-3);
        assertEquals("[static rotate] wrong yaw value", MathFunctions.adjustAngleP((float) expected.get(2)), result.yaw, 1E-3);
        assertEquals("[static rotate] wrong vx value", vx1, result.vx, ERR);
        assertEquals("[static rotate] wrong vy value", vy1, result.vy, ERR);
        assertEquals("[static rotate] wrong w value", w1, result.w, ERR);
    }
    
}

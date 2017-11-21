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

import es.usc.citius.lab.motionplanner.core.spatial.*;
import es.usc.citius.lab.motionplanner.core.util.MathFunctions;
import es.usc.citius.lab.motionplanner.core.util.Pair;
import org.apache.commons.configuration.HierarchicalConfiguration;
import org.apache.commons.math3.util.FastMath;

import java.util.HashMap;
import java.util.Map;

/**
 * Created by adrian.gonzalez on 19/04/17.
 */
public class ShapeSquare2DNonSimmetric extends Shape2D{

    private static final String SUBID_POSX = SUBID_PARAM + ".positiveX";
    private static final String SUBID_NEGX = SUBID_PARAM + ".negativeX";
    private static final String SUBID_POSY = SUBID_PARAM + ".positiveY";
    private static final String SUBID_NEGY = SUBID_PARAM + ".negativeY";
    private Map<Integer, Point2D> borderPositionsByAngle; //stores the points of the border of the shape, by angle, in degree precision
    private Map<Integer, Float> distancesByAngle; //stores the distances to the border of the shape, by angle, in degegree precision
    private Point2D[] corners; //stores the corners of the shape
    private float positiveX;
    private float positiveY;
    private float negativeX;
    private float negativeY;
    private float angle1; //angle of the front-left corner
    private float angle2; //angle of the back-left corner
    private float angle3; //angle of the back-right corner
    private float angle4; //angle of the front-right corner
    private float optimisticRadius;
    private float pessimisticRadius;

    /**
     * Initializes the squared shape based on the dimensions from its rotation center.
     *
     * @param negativeX length of the back side
     * @param negativeY length of the right side
     * @param positiveX length of the left side
     * @param positiveY length of the front side
     */
    public ShapeSquare2DNonSimmetric(float negativeX, float negativeY, float positiveX, float positiveY){
        this.negativeX = negativeX;
        this.negativeY = negativeY;
        this.positiveX = positiveX;
        this.positiveY = positiveY;
    }

    /**
     * Initializes the rectangular shape based on its dimensions from the rotation center.
     *
     * @param config configuration of the shape
     */
    public ShapeSquare2DNonSimmetric(HierarchicalConfiguration config) {
        super(config);
        initialize();
    }

    private void initialize(){
        //assign optimistic and pessimistic radius
        float radiusX = FastMath.max(FastMath.abs(positiveX), FastMath.abs(negativeX));
        float radiusY = FastMath.max(FastMath.abs(positiveY), FastMath.abs(negativeY));
        this.optimisticRadius = FastMath.min(radiusX, radiusY);
        this.pessimisticRadius = (float) FastMath.hypot(radiusX, radiusY);
        //assign the corner angles
        this.angle1 = (float) FastMath.atan2(positiveY, positiveX);
        this.angle2 = (float) FastMath.atan2(positiveY, negativeX);
        this.angle3 = (float) FastMath.atan2(negativeY, negativeX);
        this.angle4 = (float) FastMath.atan2(negativeY, positiveX);
        //generates the distances to the border by angle (from -PI/2 to PI/2)
        this.borderPositionsByAngle = new HashMap<Integer, Point2D>(360);
        this.distancesByAngle = new HashMap<Integer, Float>(360);
        Point2D p1 = new Point2D(1, 0); //rotation basis to generate the straight line
        //from -179 to 180 would be enough, but -180 is added to avoid
        //extra comparisons in roundToDegrees(float) method
        for(int i = -180; i <= 180; i++){
            //convert angle to radians
            float angle = MathFunctions.degToRadians(i);
            //create straight line of the side at that angle
            Pair<Point2D, Point2D> side = sideOfAngle(Pose2D.ZERO, angle);
            //rotated basis
            Point2D p2 = p1.rotate(angle, 0f, 0f);
            //select point of intersection
            Point2D intersection = SpatialFunctions.getLineLineIntersection(Point2D.ZERO, p2, side.getKey(), side.getContent());
            //add position in the border to the map, with degree resolution
            borderPositionsByAngle.put(i, intersection);
            //add distance to the map, with degree resolution
            distancesByAngle.put(i, intersection.distance(Point2D.ZERO));
        }
        //obtain the corners of the shape
        this.corners = new Point2D[]{
                new Point2D(positiveX, negativeY),
                new Point2D(positiveX, positiveY),
                new Point2D(negativeX, positiveY),
                new Point2D(negativeX, negativeY)
        };
    }

    /**
     * Returns the straight line of the robot border in a concrete orientation. The angle is the orientation relative
     * to the heading of the robot that is the object of the query. Yaw is the orientation of the robot pose, to rotate
     * the straight line to match with that orientation of the robot.
     *
     * @param pose
     * @param angle relative orientation to the heading to retrieve the side (in radians)
     * @return straight line of the side of the robot correspondent to the angle, rotated to match the robot pose
     */
    public Pair<Point2D, Point2D> sideOfAngle(Pose pose, float angle){
        float yaw = pose.getYaw();
        //begin and end are selected depending on the relative angle to the heading of the robot,
        //because they are the points of the corners that define the segment of the robot side
        Point2D p1, p2;
        //front side
        if(angle >= angle4 && angle < angle1){
            p1 = new Point2D( positiveX, negativeY);
            p2 = new Point2D( positiveX, positiveY);
        }
        //left side
        else if(angle >= angle1 && angle < angle2){
            p1 = new Point2D( positiveX, positiveY);
            p2 = new Point2D( negativeX, positiveY);
        }
        //right side
        else if(angle >= angle3 && angle < angle1){
            p1 = new Point2D( negativeX, negativeY);
            p2 = new Point2D( positiveX, negativeY);
        }
        //back side: equivalent to: angle >= angle2 || angle < angle3
        else{
            p1 = new Point2D( negativeX, positiveY);
            p2 = new Point2D( negativeX, negativeY);
        }

        //now the points are rotated to make the sides of the robot match with the heading
        //in global coordinates
        p1.staticRotate(yaw, 0f, 0f);
        p2.staticRotate(yaw, 0f, 0f);
        p1.staticAdd(pose);
        p2.staticAdd(pose);
        //now the straight line is built
        return new Pair<Point2D, Point2D>(p1, p2);
    }

    @Override
    public double[][] distanceVectorToPoint(Pose robotPose, Point point, float angle){
        //select side of the robot depending on the relative angle
        Pair<Point2D, Point2D> side = sideOfAngle(robotPose, angle);
        //project point over the straight line defined by the segment
        Point2D pointProjectedOverSide = new Point2D(point.getX(), point.getY()).projectOverSegment(side.getKey(), side.getContent());
        //return the distance vector to the point, where the origin is the projection
        return new double[][]{{point.getX() - pointProjectedOverSide.x}, {point.getY() - pointProjectedOverSide.y}};
    }

    @Override
    protected final void loadConfig(HierarchicalConfiguration config) {
        this.positiveX = config.getFloat(SUBID_POSX, Float.NaN);
        if(Float.isNaN(positiveX)){
            throw new RuntimeException("required field " + SUBID_POSX + " is empty");
        }
        this.negativeX = -config.getFloat(SUBID_NEGX, Float.NaN);
        if(Float.isNaN(negativeX)){
            throw new RuntimeException("required field " + SUBID_NEGX + " is empty");
        }
        this.positiveY = config.getFloat(SUBID_POSY, Float.NaN);
        if(Float.isNaN(positiveY)){
            throw new RuntimeException("required field " + SUBID_POSY + " is empty");
        }
        this.negativeY = -config.getFloat(SUBID_NEGY, Float.NaN);
        if(Float.isNaN(negativeY)){
            throw new RuntimeException("required field " + SUBID_NEGY + " is empty");
        }
    }

    @Override
    public float getMinRadius() {
        return optimisticRadius;
    }

    @Override
    public float getMaxRadius() {
        return pessimisticRadius;
    }

    /**
     * Obtains the rounded angle in degrees.
     *
     * @param angle in rad (-pi, pi]
     * @return
     */
    private int roundDegrees(float angle){
        return FastMath.round(MathFunctions.radiansToDeg(angle));
    }

    @Override
    public Point2D borderPointAtRelativeAngle(float yaw, float pitch) {
        return borderPositionsByAngle.get(roundDegrees(yaw));
    }

    @Override
    public float borderDistanceAtRelativeAngle(float yaw, float pitch) {
        return distancesByAngle.get(roundDegrees(yaw));
    }

    @Override
    public Vector2D[] axisAt(Pose pose) {
        float yaw = pose.getYaw();
        //pre-calculated values for effiency
        float cos = (float) FastMath.cos(yaw);
        float sin = (float) FastMath.sin(yaw);
        return new Vector2D[]{
                new Vector2D(positiveX * cos, positiveX * sin),
                new Vector2D(negativeY * sin, positiveY * cos)
        };
    }

    /**
     * Returns the position of the corners of the robot shape centered in the
     * pose specified.
     *
     * @param pose pose of the robot shape
     * @return list of {@link Point2D} with the corners of the shape when it is centered in the given pose
     */
    @Override
    public Point2D[] vertexAt(Pose pose) {
        float x = pose.getX();
        float y = pose.getY();
        float yaw = pose.getYaw();
        //pre-calculated values for effiency
        float cos = (float) FastMath.cos(yaw);
        float sin = (float) FastMath.sin(yaw);
        float negativeXDotCos = negativeX * cos;
        float positiveXDotCos = positiveX * cos;
        float negativeXDotSin = negativeX * sin;
        float positiveXDotSin = positiveX * sin;
        float negativeYDotCos = negativeY * cos;
        float positiveYDotCos = positiveY * cos;
        float negativeYDotSin = negativeY * sin;
        float positiveYDotSin = positiveY * sin;
        //calculate corners of a squared shape, first rotated and then added according to the pose information
        return new Point2D[]{
                new Point2D(x + positiveXDotCos - negativeYDotSin, y + positiveXDotSin + negativeYDotCos),
                new Point2D(x + positiveXDotCos - positiveYDotSin, y + positiveXDotSin + positiveYDotCos),
                new Point2D(x + negativeXDotCos - positiveYDotSin, y + negativeXDotSin + positiveYDotCos),
                new Point2D(x + negativeXDotCos - negativeYDotSin, y + negativeXDotSin + negativeYDotCos)
        };
    }
}

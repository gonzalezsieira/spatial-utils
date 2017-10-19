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

import es.usc.citius.lab.motionplanner.core.spatial.Point2D;

import java.util.*;

import es.usc.citius.lab.motionplanner.core.spatial.Vector2D;
import org.apache.commons.math3.util.FastMath;

import es.usc.citius.lab.motionplanner.core.spatial.Pose2D;
import es.usc.citius.lab.motionplanner.core.spatial.SpatialFunctions;
import es.usc.citius.lab.motionplanner.core.util.MathFunctions;
import es.usc.citius.lab.motionplanner.core.util.Pair;
import org.apache.commons.configuration.HierarchicalConfiguration;

/**
 * This class contains a set of points that defines, approximately, the form of
 * the shape.
 *
 * @author Adrián González Sieira <a href=mailto:adrian.gonzalez@usc.es>adrian.gonzalez@usc.es</a>
 */
public final class ShapeSquare2D extends Shape2D {

    private static final String SUBID_DIMX = SUBID_PARAM + ".dimX";
    private static final String SUBID_DIMY = SUBID_PARAM + ".dimY";
    private static final long serialVersionUID = 201507171L;
    
    private Map<Integer, Point2D> borderPositionsByAngle; //stores the points of the border of the shape, by angle, in degree precision
    private Map<Integer, Float> distancesByAngle; //stores the distances to the border of the shape, by angle, in degegree precision
    private float halfDimX;
    private float halfDimY;
    private float angleFront; //angle of the front-left corner
    private float angleBack; //angle of the back-left corner
    private float optimisticRadius;
    private float pessimisticRadius;

    /**
     * Default constructor for bidimensional squared shape.
     * 
     * @param dimX maximum lenght of the shape in the X axis
     * @param dimY maximum lenght of the shape in the Y axis
     */
    public ShapeSquare2D(float dimX, float dimY) {
        //assign the dimensions
        this.halfDimX = dimX / 2;
        this.halfDimY = dimY / 2;
        initialize();
    }
    
    /**
     * Builds an instance of {@link ShapeSquare2D} based on the 
     * information of a {@code <shape>...</shape>}
     * 
     * @param config information of the {@code <shape>...</shape>} in XML format
     */
    public ShapeSquare2D(HierarchicalConfiguration config){
        loadConfig(config);
        initialize();
    }

    @Override
    protected final void loadConfig(HierarchicalConfiguration config) {
        this.halfDimX = config.getFloat(SUBID_DIMX, Float.NaN) / 2;
        if(Float.isNaN(halfDimX)){
            throw new RuntimeException("required field " + SUBID_DIMX + " is empty");
        }
        this.halfDimY = config.getFloat(SUBID_DIMY, Float.NaN) / 2;
        if(Float.isNaN(halfDimY)){
            throw new RuntimeException("required field " + SUBID_DIMY + " is empty");
        }
    }
    
    /**
     * Initializes the params of this class that are calculated from the "halfDimX" and
     * "halfDimY" fields.
     */
    private void initialize(){
        //assign optimistic and pessimistic radius
        this.optimisticRadius = FastMath.min(halfDimX, halfDimY);
        this.pessimisticRadius = (float) FastMath.hypot(halfDimX, halfDimY);
        //assign the corner angles
        this.angleFront = (float) FastMath.atan2(halfDimY, halfDimX);
        this.angleBack = (float) FastMath.atan2(halfDimY, -halfDimX);
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
            Point2D p2 = Point2D.rotate(p1, angle);
            //select point of intersection
            Point2D intersection = SpatialFunctions.getLineLineIntersection(Point2D.ZERO, p2, side.getKey(), side.getContent());
            //add position in the border to the map, with degree resolution
            borderPositionsByAngle.put(i, intersection);
            //add distance to the map, with degree resolution
            distancesByAngle.put(i, intersection.distance(Point2D.ZERO));
        }
    }

    /**
     * Returns the position of the corners of the robot shape centered in the
     * pose specified.
     *
     * @param pose pose of the robot shape
     * @return list of {@link Point2D} with the corners of the shape when it is centered in the given pose
     */
    @Override
    public Point2D[] vertexAt(Pose2D pose) {
        //pre-calculated values for effiency
        float cos = (float) FastMath.cos(pose.yaw);
        float sin = (float) FastMath.sin(pose.yaw);
        float halfDimXDotCos = halfDimX * cos;
        float halfDimYDotCos = halfDimY * cos;
        float halfDimXDotSin = halfDimX * sin;
        float halfDimYDotSin = halfDimY * sin;
        //calculate corners of a squared shape, first rotated and then added according to the pose information
        return new Point2D[]{
                new Point2D(pose.x + halfDimXDotCos + halfDimYDotSin, pose.y + halfDimXDotSin - halfDimYDotCos),
                new Point2D(pose.x + halfDimXDotCos - halfDimYDotSin, pose.y + halfDimXDotSin + halfDimYDotCos),
                new Point2D(pose.x - halfDimXDotCos - halfDimYDotSin, pose.y - halfDimXDotSin + halfDimYDotCos),
                new Point2D(pose.x - halfDimXDotCos + halfDimYDotSin, pose.y - halfDimXDotSin - halfDimYDotCos)
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
    public Pair<Point2D, Point2D> sideOfAngle(Pose2D pose, float angle){
    	//begin and end are selected depending on the relative angle to the heading of the robot,
    	//because they are the points of the corners that define the segment of the robot side
        Point2D p1, p2;
        //front side
        if(angle >= -angleFront && angle < angleFront){
            p1 = new Point2D( halfDimX, -halfDimY);
            p2 = new Point2D( halfDimX, halfDimY);
        }        
        //left side
        else if(angle >= angleFront && angle < angleBack){
            p1 = new Point2D( halfDimX, halfDimY);
            p2 = new Point2D( -halfDimX, halfDimY);
        }      
        //right side
        else if(angle >= -angleBack && angle < -angleFront){
            p1 = new Point2D( -halfDimX, -halfDimY); 
            p2 = new Point2D( halfDimX, -halfDimY);
        }
        //back side: equivalent to: angle >= angleBack || angle < -angleBack
        else{
            p1 = new Point2D( -halfDimX, halfDimY);
            p2 = new Point2D( -halfDimX, -halfDimY);
        }

        //now the points are rotated to make the sides of the robot match with the heading
        //in global coordinates
        p1.staticRotate(pose.yaw);
        p2.staticRotate(pose.yaw);
        p1.staticAdd(pose);
        p2.staticAdd(pose);
        //now the straight line is built
        return new Pair<Point2D, Point2D>(p1, p2);
    }
    
    @Override
    public double[][] distanceVectorToPoint(Pose2D robotPose, Point2D point, float angle){
        //select side of the robot depending on the relative angle
        Pair<Point2D, Point2D> side = sideOfAngle(robotPose, angle);
        //project point over the straight line defined by the segment
        Point2D pointProjectedOverSide = point.projectOverSegment(side.getKey(), side.getContent());
        //return the distance vector to the point, where the origin is the projection
        return new double[][]{{point.getX() - pointProjectedOverSide.getX()}, {point.getY() - pointProjectedOverSide.getY()}};
    }

    public float getDimX() {
        return halfDimX * 2;
    }

    public float getDimY() {
        return halfDimY * 2;
    }

    @Override
    public Point2D borderPointAtRelativeAngle(float angle) {
        return borderPositionsByAngle.get(roundDegrees(angle));
    }

    @Override
    public float borderDistanceAtRelativeAngle(float angle) {
        return distancesByAngle.get(roundDegrees(angle));
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
    public float getMinRadius() {
        return optimisticRadius;
    }

    @Override
    public float getMaxRadius() {
        return pessimisticRadius;
    }

    @Override
    public Vector2D[] axisAt(Pose2D pose) {
        //pre-calculated values for effiency
        float cos = (float) FastMath.cos(pose.yaw);
        float sin = (float) FastMath.sin(pose.yaw);
        return new Vector2D[]{
                new Vector2D(halfDimX * cos, halfDimX * sin),
                new Vector2D(-halfDimY * sin, halfDimY * cos)
        };
    }
}
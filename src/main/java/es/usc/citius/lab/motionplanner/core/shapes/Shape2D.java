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

/**
 * This class defines a main methods to interact with a definition of 
 * robot shape. Different subclasses can be defined, as Circle, Rectangle, etc. The
 * different subclasses will use the definition of Shape to interact with other
 * classes, but the form of manage the data is different.
 *
 * @author Adrián González Sieira <<a href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>>
 */
public abstract class Shape2D extends Shape{

    private static final long serialVersionUID = 201507171L;

    /**
     * Retrieves the position of the border of this shape in a relative
     * orientation from the heading.
     * 
     * @param yaw relative orientation from heading
     * @return 2D position of the border of the at the given orientation
     */
    public abstract Point2D borderPointAtRelativeAngle(float yaw);
    
    /**
     * Retrieves the distance between the border of the shape  of the border
     * 
     * @param yaw relative orientation from heading
     * @return distance between the rotation center and the border at the given orientation
     */
    public abstract float borderDistanceAtRelativeAngle(float yaw);

    /**
     * Retuns an interable set with the vertex of the shape,
     * given the pose of the rotation center.
     *
     * @param pose of the rotation center of the shape
     *
     * @return iterable set of vertex
     */
    public abstract Point2D[] vertexAt(Pose pose);

    /**
     * Retuns an interable set with the axis of the shape,
     * given the pose of the rotation center.
     *
     * @param pose of the rotation center of the shape
     *
     * @return iterable set of axis
     */
    public abstract Vector2D[] axisAt(Pose pose);

    /**
     * Calculates the distance vector between the border of the shape 
     * and the point given as argument; this method calls
     * {@link #distanceVectorToPoint(Pose, Point, float) }.
     * 
     * @param robotPose current robot pose
     * @param point point in the map
     * @return column vector distance to the point where the origin is the robot pose: [dx; dy]
     */
    public double[][] distanceVectorToPoint(Pose robotPose, Point point){
        return distanceVectorToPoint(robotPose, point, MathFunctions.adjustAngleP(robotPose.yawTo(point) - robotPose.getYaw()));
    }
    
    /**
     * Calculates exactly the distance vector from the robot to a point in the map; the distance vector
     * is calculated taking into account the robot shape.
     * 
     * @param robotPose current robot pose
     * @param point point in the map
     * @param angle difference of angles between heading and {@link Point#yawTo(Point)}  }
     * @return column vector distance to the point where the origin is the robot pose: [dx; dy]
     */
    public abstract double[][] distanceVectorToPoint(Pose robotPose, Point point, float angle);

}

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
import es.usc.citius.lab.motionplanner.core.spatial.Pose2D;

import java.io.Serializable;
import java.lang.reflect.InvocationTargetException;

import es.usc.citius.lab.motionplanner.core.spatial.Vector2D;
import org.apache.commons.configuration.HierarchicalConfiguration;

/**
 * This class defines a main methods to interact with a definition of 
 * robot shape. Different subclasses can be defined, as Circle, Rectangle, etc. The
 * different subclasses will use the definition of Shape to interact with other
 * classes, but the form of manage the data is different.
 *
 * @author Adrián González Sieira <<a href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>>
 */
public abstract class Shape2D implements Serializable{

    protected static final String SUBID_CLASS = "class";
    protected static final String SUBID_PARAM = "parameters";
    private static final long serialVersionUID = 201507171L;
    
    /**
     * Loads the information of the shape contained in a 
     * {@link HierarchicalConfiguration} file.
     * 
     * @param config 
     */
    protected abstract void loadConfig(HierarchicalConfiguration config);
    
    /**
     * Retrieves the position of the border of this shape in a relative
     * orientation from the heading.
     * 
     * @param angle relative orientation from heading
     * @return 2D position of the border of the at the given orientation
     */
    public abstract Point2D borderPointAtRelativeAngle(float angle);
    
    /**
     * Retrieves the distance between the border of the shape  of the border
     * 
     * @param angle relative orientation from heading
     * @return distance between the rotation center and the border at the given orientation
     */
    public abstract float borderDistanceAtRelativeAngle(float angle);

    /**
     * Retuns an interable set with the vertex of the shape,
     * given the pose of the rotation center.
     * 
     * @param pose of the rotation center of the shape
     * 
     * @return iterable set of vertex
     */
    public abstract Point2D[] vertexAt(Pose2D pose);

    /**
     * Retuns an interable set with the axis of the shape,
     * given the pose of the rotation center.
     *
     * @param pose of the rotation center of the shape
     *
     * @return iterable set of axis
     */
    public abstract Vector2D[] axisAt(Pose2D pose);
    
    /**
     * Retrieves the optimistic radius of the robot shape.
     * 
     * @return 
     */
    public abstract float getMinRadius();
    
    /**
     * Retrieves the pessimistic radius of the robot shape.
     * 
     * @return 
     */
    public abstract float getMaxRadius();
    
    /**
     * Calculates the distance vector between the border of the shape 
     * and the point given as argument; this method calls
     * {@link #distanceVectorToPoint(Pose2D, Point2D, float) }, passing
     * as third argument the value returned by 
     * {@link Pose2D#relativeYawTo(Point2D) }.
     * 
     * @param robotPose current robot pose
     * @param point point in the map
     * @return column vector distance to the point where the origin is the robot pose: [dx; dy]
     */
    public double[][] distanceVectorToPoint(Pose2D robotPose, Point2D point){
        return distanceVectorToPoint(robotPose, point, robotPose.relativeYawTo(point));
    }
    
    /**
     * Calculates exactly the distance vector from the robot to a point in the map; the distance vector
     * is calculated taking into account the robot shape.
     * 
     * @param robotPose current robot pose
     * @param point point in the map
     * @param angle value given by {@link Pose2D#relativeYawTo(es.usc.citius.lab.motionplanner.core.spatial.Point2D) }
     * @return column vector distance to the point where the origin is the robot pose: [dx; dy]
     */
    public abstract double[][] distanceVectorToPoint(Pose2D robotPose, Point2D point, float angle);
    
    /**
     * Obtains a new instance based on the configuration passed as an 
     * argument.
     * 
     * @param config {@code <shape>...</shape>} configuration group
     * @return instance of {@link Shape2D} with the parameters specified
     */
    public static Shape2D create(HierarchicalConfiguration config){
        //retrieve class of the shape
        String className = config.getString(SUBID_CLASS, "");
        if(className.isEmpty()){
            throw new RuntimeException("required value robot.shape.class is missing");
        }
        try{
            return (Shape2D) ClassLoader.getSystemClassLoader().loadClass(className).getDeclaredConstructor(HierarchicalConfiguration.class).newInstance(config);
        } catch(ClassNotFoundException ex) {
            throw new RuntimeException("class " + className + " cannot be found: " + ex);
        } catch(NoSuchMethodException ex) {
            throw new RuntimeException("referenced class " + className + " does not implement new(HierarchicalConfiguration): " + ex);
        } catch(InstantiationException ex){
            throw new RuntimeException("referenced class " + className + " is not instantiable: " + ex);
        } catch(IllegalAccessException ex){
            throw new RuntimeException("constructor new(HierarchicalConfiguration) of the class " + className + " is not accesible: " + ex);
        } catch(InvocationTargetException ex){
            throw new RuntimeException("Internal error in the constructor of the class: " + className + "; " + ex.getTargetException().toString());
        }
    }
    
}

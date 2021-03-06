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

import es.usc.citius.lab.motionplanner.core.spatial.Point;
import es.usc.citius.lab.motionplanner.core.spatial.Point3D;
import es.usc.citius.lab.motionplanner.core.spatial.Pose;
import es.usc.citius.lab.motionplanner.core.spatial.Vector3D;
import org.apache.commons.configuration.HierarchicalConfiguration;
import org.ejml.data.FixedMatrix3x3_64F;

import java.io.Serializable;
import java.lang.reflect.InvocationTargetException;
import java.util.Vector;

/**
 *
 *
 * @author Adrián González Sieira <<a href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>>
 */
public abstract class Shape implements Serializable {

    protected static final String SUBID_CLASS = "class";
    protected static final String SUBID_PARAM = "parameters";
    private static final long serialVersionUID = 20170822L;

    /**
     * Retuns an interable set with the vertex of the shape,
     * given the pose of the rotation center.
     *
     * @param pose of the rotation center of the shape
     *
     * @return iterable set of vertex
     */
    public abstract Vector3D[] axisAt(Pose pose);

    /**
     * Retuns an interable set with the axis of the shape,
     * given the pose of the rotation center.
     *
     * @param pose of the rotation center of the shape
     *
     * @return iterable set of axis
     */
    public abstract Point3D[] vertexAt(Pose pose);

    /**
     * Returns a 3x3 matrix with the axis of the shape given the pose
     * of the rotation center.
     *
     * @param pose pose of the rotation center of the shape
     * @return 3x3 matrix with the axes
     */
    public abstract FixedMatrix3x3_64F axesMatrixAt(Pose pose);

    public abstract Vector3D distanceBetweenCenterandCentroid(Pose pose);

    /**
     * @return distance from the centroid to the shape of the border (dimension X in local frame)
     */
    public abstract double distanceToCentroidX();

    /**
     * @return distance from the centroid to the shape of the border (dimension Y in local frame)
     */
    public abstract double distanceToCentroidY();

    /**
     * @return distance from the centroid to the shape of the border (dimension Z in local frame)
     */
    public abstract double distanceToCentroidZ();

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
     * Retrieves the position of the border of this shape in a relative
     * orientation from the heading.
     *
     * @param yaw relative orientation from heading
     * @return position of the border of the at the given orientation
     */
    public abstract Point borderPointAtRelativeAngle(float yaw, float pitch);

    /**
     * Retrieves the distance between the border of the shape  of the border
     *
     * @param yaw relative orientation from heading
     * @return distance between the rotation center and the border at the given orientation
     */
    public abstract float borderDistanceAtRelativeAngle(float yaw, float pitch);

    /**
     * Loads the information of the shape contained in a
     * {@link HierarchicalConfiguration} file.
     *
     * @param config
     */
    protected abstract void loadConfig(HierarchicalConfiguration config);

    /**
     * Obtains a new instance based on the configuration passed as an
     * argument.
     *
     * @param config {@code <shape>...</shape>} configuration group
     * @return instance of {@link Shape} with the parameters specified
     */
    public static Shape create(HierarchicalConfiguration config){
        //retrieve class of the shape
        String className = config.getString(Shape.SUBID_CLASS, "");
        if(className.isEmpty()){
            throw new RuntimeException("required value robot.shape.class is missing");
        }
        try{
            return (Shape) ClassLoader.getSystemClassLoader().loadClass(className).getDeclaredConstructor(HierarchicalConfiguration.class).newInstance(config);
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

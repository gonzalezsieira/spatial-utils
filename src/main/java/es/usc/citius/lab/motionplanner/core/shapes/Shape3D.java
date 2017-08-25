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

/**
 *
 *
 * @author Adrián González Sieira <<a href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>>
 */
public abstract class Shape3D extends Shape {

    protected static final String SUBID_CLASS = "class";
    protected static final String SUBID_PARAM = "parameters";
    private static final long serialVersionUID = 20170823L;

    /**
     * Retrieves the position of the border of this shape in a relative
     * orientation from the heading.
     *
     * @param yaw relative orientation from heading
     * @return 2D position of the border of the at the given orientation
     */
    public abstract Point3D borderPointAtRelativeAngle(float yaw, float pitch);

    /**
     * Retrieves the distance between the border of the shape  of the border
     *
     * @param yaw orientation around Z (between -PI and PI)
     * @param pitch orientation around Y (between -PI/4 and PI/4)
     * @return distance between the rotation center and the border at the given orientation
     */
    public abstract float borderDistanceAtRelativeAngle(float yaw, float pitch);

    /**
     * Retuns an interable set with the vertex of the shape,
     * given the pose of the rotation center.
     *
     * @param pose of the rotation center of the shape
     *
     * @return iterable set of vertex
     */
    public abstract Point3D[] vertexAt(Pose3D pose);

    /**
     * Retuns an interable set with the axis of the shape,
     * given the pose of the rotation center.
     *
     * @param pose of the rotation center of the shape
     *
     * @return iterable set of axis
     */
    public abstract Vector3D[] axisAt(Pose3D pose);

}

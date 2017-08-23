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

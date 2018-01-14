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

import org.apache.commons.configuration.HierarchicalConfiguration;
import org.apache.commons.math3.util.FastMath;
import org.ejml.data.FixedMatrix2x2_64F;
import org.ejml.data.FixedMatrix3x3_64F;

/**
 * This class implements a specific form of the shape, that defines a circle
 * with the center in a point with a radius defined in each instance.
 *
 * @author Adrián González Sieira <a href=mailto:adrian.gonzalez@usc.es>adrian.gonzalez@usc.es</a>
 */
public final class ShapeCircle2D extends Shape2D {

    private static final String SUBID_RADIUS = SUBID_PARAM + ".radius";
    private float radius;
    private static final long serialVersionUID = 201507171L;

    /**
     * The constructor builds a circular shape with a given radius.
     *
     * @param radius distance from the center to the limit of the circle
     */
    public ShapeCircle2D(float radius) {
        super();
        this.radius = radius;
    }

    /**
     * Builds an instance of {@link ShapeCircle2D} based on the 
     * information of a {@code <shape>...</shape>}
     * 
     * @param config information of the {@code <shape>...</shape>} in XML format
     */
    public ShapeCircle2D(HierarchicalConfiguration config){
        super(config);
    }
    
    @Override
    protected final void loadConfig(HierarchicalConfiguration config) {
        this.radius = config.getFloat(SUBID_RADIUS, Float.NaN);
        if(Float.isNaN(radius)){
            throw new RuntimeException("required field " + SUBID_RADIUS + " is empty");
        }
    }

    public float getRadius() {
        return radius;
    }

    @Override
    public Point2D borderPointAtRelativeAngle(float yaw, float pitch) {
        float x = (float) FastMath.cos(yaw) * radius;
        float y = (float) FastMath.sin(yaw) * radius;
        return new Point2D(x, y);
    }

    @Override
    public float borderDistanceAtRelativeAngle(float yaw, float pitch) {
        return radius;
    }

    @Override
    public Point3D[] vertexAt(Pose pose) {
        return new Point3D[]{
                new Point3D(pose.getX() + radius, pose.getY(), 0f),
                new Point3D(pose.getX() - radius, pose.getY(), 0f),
                new Point3D(pose.getX(), pose.getY() + radius, 0f),
                new Point3D(pose.getX(), pose.getY() - radius, 0f)
        };
    }

    @Override
    public float getMinRadius() {
        return radius;
    }

    @Override
    public float getMaxRadius() {
        return radius;
    }

    @Override
    public double[][] distanceVectorToPoint(Pose robotPose, Point point, float yaw) {
        Point2D border = borderPointAtRelativeAngle(yaw, 0f);
        return new double[][]{{point.getX() - border.getX()}, {point.getY() - border.getY()}};
    }

    @Override
    public Vector3D[] axisAt(Pose pose) {
        return new Vector3D[]{
            Vector3D.X,
            Vector3D.Y
        };
    }

    @Override
    public FixedMatrix3x3_64F axesMatrixAt(Pose pose) {
        FixedMatrix3x3_64F matrix = new FixedMatrix3x3_64F();
        matrix.a11 = 1; //1st column: (1, 0, 0)
        matrix.a22 = 1; //2nd column: (0, 1, 0) //3rd column: (0, 0, 0)
        return matrix;
    }

    @Override
    public FixedMatrix2x2_64F axesMatrix2DAt(Pose pose) {
        FixedMatrix2x2_64F matrix = new FixedMatrix2x2_64F();
        matrix.a11 = 1; //1st column: (1, 0, 0)
        matrix.a22 = 1; //2nd column: (0, 1, 0) //3rd column: (0, 0, 0)
        return matrix;
    }

    @Override
    public double distanceToCentroidX() {
        return radius;
    }

    @Override
    public double distanceToCentroidY() {
        return radius;
    }

    @Override
    public double distanceToCentroidZ() {
        return radius;
    }

    @Override
    public Vector3D distanceBetweenCenterandCentroid(Pose pose) {
        return Vector3D.ZERO;
    }
}

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
    public Point2D borderPointAtRelativeAngle(float angle) {
        float x = (float) FastMath.cos(angle) * radius;
        float y = (float) FastMath.sin(angle) * radius;
        return new Point2D(x, y);
    }

    @Override
    public float borderDistanceAtRelativeAngle(float yaw) {
        return radius;
    }

    @Override
    public Point2D[] vertexAt(Pose pose) {
        return new Point2D[]{
                new Point2D(pose.getX() + radius, pose.getY()),
                new Point2D(pose.getX() - radius, pose.getY()),
                new Point2D(pose.getX(), pose.getY() + radius),
                new Point2D(pose.getX(), pose.getY() - radius)
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
    public double[][] distanceVectorToPoint(Pose robotPose, Point point, float angle) {
        Point2D border = borderPointAtRelativeAngle(angle);
        return new double[][]{{point.getX() - border.getX()}, {point.getY() - border.getY()}};
    }

    @Override
    public Vector2D[] axisAt(Pose pose) {
        return new Vector2D[]{
            Vector2D.X,
            Vector2D.Y
        };
    }
}

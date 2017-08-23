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

import es.usc.citius.lab.motionplanner.core.spatial.Point3D;
import es.usc.citius.lab.motionplanner.core.spatial.Pose3D;
import es.usc.citius.lab.motionplanner.core.spatial.Vector3D;
import org.apache.commons.configuration.HierarchicalConfiguration;

/**
 * This class implements a specific form of {@link Shape3D} that defines a sphere
 * with the center in a {@link Point3D} with a radius.
 *
 * @author Adrián González Sieira <a href=mailto:adrian.gonzalez@usc.es>adrian.gonzalez@usc.es</a>
 */
public class ShapeSphere3D extends Shape3D{

    private static final String SUBID_RADIUS = Shape.SUBID_PARAM + ".radius";
    private float radius;

    /**
     * The constructor builds a spherical shape with a given radius.
     *
     * @param radius distance from the center to the limit of the sphere
     */
    public ShapeSphere3D(float radius){
        this.radius = radius;
    }

    @Override
    public Point3D borderPointAtRelativeAngle(float yaw, float pitch) {
        //create point in the border of the sphere
        Point3D border = new Point3D(radius, 0, 0);
        //rotate point accordingly
        border.rotate(yaw, pitch, 0f);
        return border;
    }

    @Override
    public float borderDistanceAtRelativeAngle(float yaw, float pitch) {
        return radius;
    }

    @Override
    public Point3D[] vertexAt(Pose3D pose) {
        return new Point3D[]{
                new Point3D(pose.x + radius, pose.y, pose.z),
                new Point3D(pose.x - radius, pose.y, pose.z),
                new Point3D(pose.x, pose.y + radius, pose.z),
                new Point3D(pose.x, pose.y - radius, pose.z),
                new Point3D(pose.x, pose.y, pose.z + radius),
                new Point3D(pose.x, pose.y, pose.z - radius)
        };
    }

    @Override
    public Vector3D[] axisAt(Pose3D pose) {
        return new Vector3D[]{
            Vector3D.X,
            Vector3D.Y,
            Vector3D.Z
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
    protected void loadConfig(HierarchicalConfiguration config) {
        this.radius = config.getFloat(SUBID_RADIUS, Float.NaN);
        if(Float.isNaN(radius)){
            throw new RuntimeException("required field " + SUBID_RADIUS + " is empty");
        }
    }
}

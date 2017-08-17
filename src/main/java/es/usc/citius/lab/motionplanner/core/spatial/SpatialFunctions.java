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
package es.usc.citius.lab.motionplanner.core.spatial;

/**
 * This class contains the following spatial operations:
 * <ul>
 * <li>Intersection point between a plane and a 3D line</li>
 * <li>Intersection point between 2D lines</li>
 * </ul>
 *
 * @author Adrián González Sieira <a
 * href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>
 */
public class SpatialFunctions {

    /**
     * Performs the intersection between a point and a plane in 3D coordinates,
     * as detailed in
     * http://math.stackexchange.com/questions/83990/line-and-plane-intersection-in-3d.
     *
     * @param a first point of the 3D line
     * @param b second point of the 3D line
     * @param normal normal vector to the plane
     * @param distance distance between the plane and the point (0, 0, 0)
     * @return intersection between the line and the plane
     */
    public static Point3D intersectionBetweenLineAndPlane(Point3D a, Point3D b, Vector3D normal, float distance) {
        //a mult normal
        float normalDotA = a.x * normal.x + a.y * normal.y + a.z * normal.z;
        //b - a vector components
        float baX = b.x - a.x;
        float baY = b.y - a.y;
        float baZ = b.z - a.z;
        //b - a mult normal
        float normalDotBA = baX * normal.x + baY * normal.y + baZ * normal.z;
        //projection index
        float projIndex = (distance - normalDotA) / normalDotBA;
        //projection components
        float projectionX = a.x + baX * projIndex;
        float projectionY = a.y + baY * projIndex;
        float projectionZ = a.z + baZ * projIndex;
        return new Point3D(projectionX, projectionY, projectionZ);
    }
    
    /**
     * Point intersection between infinite lines; code got from
     * http://www.java-gaming.org/index.php?topic=22590.0, adapted to the data
     * model of the application.
     * 
     * @param a1 first point of the first line
     * @param a2 second poinf of the first line
     * @param b1 first point of the second line
     * @param b2 second point of the second line
     * @return {@link Point2D} of the intersection, null if they are paralell.
     */
    public static Point2D getLineLineIntersection(Point2D a1, Point2D a2, Point2D b1, Point2D b2) {
        double det1And2 = det(a1.x, a1.y, a2.x, a2.y);
        double det3And4 = det(b1.x, b1.y, b2.x, b2.y);
        double x1LessX2 = a1.x - a2.x;
        double y1LessY2 = a1.y - a2.y;
        double x3LessX4 = b1.x - b2.x;
        double y3LessY4 = b1.y - b2.y;
        double det1Less2And3Less4 = det(x1LessX2, y1LessY2, x3LessX4, y3LessY4);
        if (det1Less2And3Less4 == 0) {
            // the denominator is zero so the lines are parallel and there's either no solution (or multiple solutions if the lines overlap) so return null.
            return null;
        }
        double x = (det(det1And2, x1LessX2,
                det3And4, x3LessX4)
                / det1Less2And3Less4);
        double y = (det(det1And2, y1LessY2,
                det3And4, y3LessY4)
                / det1Less2And3Less4);
        return new Point2D((float) x, (float) y);
        
    }
    
    private static double det(double a, double b, double c, double d) {
        return a * d - b * c;
    }

}

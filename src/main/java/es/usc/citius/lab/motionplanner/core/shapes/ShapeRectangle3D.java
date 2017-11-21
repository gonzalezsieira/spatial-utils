package es.usc.citius.lab.motionplanner.core.shapes;

import es.usc.citius.lab.motionplanner.core.spatial.*;
import org.apache.commons.configuration.HierarchicalConfiguration;
import org.apache.commons.math3.util.FastMath;

import java.util.Comparator;
import java.util.PriorityQueue;

/**
 * Implements a cuboid shape for a 3D coordinate systems.
 *
 * @author Adrián González Sieira <adrian.gonzalez@usc.es>
 */
public class ShapeRectangle3D extends Shape3D{

    private float halfDimX, halfDimY, halfDimZ;
    private float minRadius, maxRadius;
    private float yawCornerFront, yawCornerBack;
    private Point3D[] CORNERS;
    private Vector3D AXIS_X = new Vector3D(1, 0, 0);
    private Vector3D AXIS_Y = new Vector3D(0, 1, 0);
    private Vector3D AXIS_Z = new Vector3D(0, 0, 1);
    private Point3D UNIT = new Point3D(1, 0, 0);

    //hierarchical configuration params
    private static final String SUBID_DIMX = SUBID_PARAM + ".dimX";
    private static final String SUBID_DIMY = SUBID_PARAM + ".dimY";
    private static final String SUBID_DIMZ = SUBID_PARAM + ".dimZ";

    /**
     * Initializes the cuboid shape from a configuration file.
     *
     * @param config {@link HierarchicalConfiguration}.
     */
    public ShapeRectangle3D(HierarchicalConfiguration config){
        super(config);
    }

    /**
     * Create a new cuboid shape. The rotation center is in the middle of the shape.
     *
     * @param dimX lenght (from back to front)
     * @param dimY width (from side to side)
     * @param dimZ altitude (from top to bottom)
     */
    public ShapeRectangle3D(float dimX, float dimY, float dimZ) {
        this.halfDimX = dimX / 2;
        this.halfDimY = dimY / 2;
        this.halfDimZ = dimZ / 2;

        //distances to border
        this.minRadius = FastMath.min(FastMath.min(halfDimX, halfDimY), halfDimZ);
        this.maxRadius = new Point3D(halfDimX, halfDimY, halfDimZ).distance(Point3D.ZERO);

        //angles to corners
        this.yawCornerFront = (float) FastMath.atan2(halfDimY, halfDimX);
        this.yawCornerBack = (float) FastMath.atan2(halfDimY, -halfDimX);

        //generate corners
        CORNERS = new Point3D[8];
        CORNERS[0] = new Point3D(halfDimX, -halfDimY, halfDimZ);
        CORNERS[0] = new Point3D(halfDimX, halfDimY, halfDimZ);
        CORNERS[0] = new Point3D(-halfDimX, halfDimY, halfDimZ);
        CORNERS[0] = new Point3D(-halfDimX, -halfDimY, halfDimZ);
        CORNERS[0] = new Point3D(halfDimX, -halfDimY, -halfDimZ);
        CORNERS[0] = new Point3D(halfDimX, halfDimY, -halfDimZ);
        CORNERS[0] = new Point3D(-halfDimX, halfDimY, -halfDimZ);
        CORNERS[0] = new Point3D(-halfDimX, -halfDimY, -halfDimZ);
    }

    @Override
    public Point3D borderPointAtRelativeAngle(float yaw, float pitch) {
        Point3D point = UNIT.rotate(yaw, pitch, 0);
        Vector3D normal;
        float distance;
        //horizontal
        if(Math.abs(pitch) < 0.01f){
            //front side
            if(yaw >= -yawCornerFront && yaw < yawCornerFront){
                normal = new Vector3D(1, 0, 0);
                distance = halfDimX;
            }
            //left side
            else if(yaw >= yawCornerFront && yaw < yawCornerBack){
                normal = new Vector3D(0, 1, 0);
                distance = halfDimY;
            }
            //right side
            else if(yaw >= -yawCornerBack && yaw < -yawCornerFront){
                normal = new Vector3D(0, 1, 0);
                distance = -halfDimY;
            }
            //back side: equivalent to: angle >= angleBack || angle < -angleBack
            else{
                normal = new Vector3D(1, 0, 0);
                distance = -halfDimX;
            }
            return SpatialFunctions.intersectionBetweenLineAndPlane(Point3D.ZERO, point, normal, distance);
        }
        //with pitch > 0 or < 0
        else{

            /**
             * Private defined class to compare distance to origin
             */
            class ComparatorDistanceToOrigin implements Comparator<Point3D>{

                @Override
                public int compare(Point3D o1, Point3D o2) {
                    return Float.compare(o1.distance(Point3D.ZERO), o2.distance(Point3D.ZERO));
                }
            }

            PriorityQueue<Point3D> queue = new PriorityQueue<Point3D>(3, new ComparatorDistanceToOrigin());

            //add top/bottom plane intersections
            if(pitch > 0) {
                //top plane
                queue.add(SpatialFunctions.intersectionBetweenLineAndPlane(Point3D.ZERO, point, new Vector3D(0, 0, 1), halfDimZ));
            }
            else {
                //bottom plane
                queue.add(SpatialFunctions.intersectionBetweenLineAndPlane(Point3D.ZERO, point, new Vector3D(0, 0, 1), -halfDimZ));
            }

            //add lateral planes
            //back-right corner
            if(yaw >= -FastMath.PI && yaw < -FastMath.PI / 2) {
                //back plane
                queue.add(SpatialFunctions.intersectionBetweenLineAndPlane(Point3D.ZERO, point, new Vector3D(1, 0, 0), -halfDimX));
                //right plane
                queue.add(SpatialFunctions.intersectionBetweenLineAndPlane(Point3D.ZERO, point, new Vector3D(0, 1, 0), -halfDimY));
            }
            //front-right corner
            else if(yaw >= -FastMath.PI / 2 && yaw < 0) {
                //front plane
                queue.add(SpatialFunctions.intersectionBetweenLineAndPlane(Point3D.ZERO, point, new Vector3D(1, 0, 0), halfDimX));
                //right plane
                queue.add(SpatialFunctions.intersectionBetweenLineAndPlane(Point3D.ZERO, point, new Vector3D(0, 1, 0), -halfDimY));
            }
            //front-left corner
            else if(yaw >= 0 && yaw < FastMath.PI / 2) {
                //front plane
                queue.add(SpatialFunctions.intersectionBetweenLineAndPlane(Point3D.ZERO, point, new Vector3D(1, 0, 0), halfDimX));
                //left plane
                queue.add(SpatialFunctions.intersectionBetweenLineAndPlane(Point3D.ZERO, point, new Vector3D(0, 1, 0), halfDimY));
            }
            //back-left corner
            else{
                //back plane
                queue.add(SpatialFunctions.intersectionBetweenLineAndPlane(Point3D.ZERO, point, new Vector3D(1, 0, 0), -halfDimX));
                //left plane
                queue.add(SpatialFunctions.intersectionBetweenLineAndPlane(Point3D.ZERO, point, new Vector3D(0, 1, 0), halfDimY));
            }
            //return point with the lowest distance
            return queue.poll();
        }
    }

    @Override
    public float borderDistanceAtRelativeAngle(float yaw, float pitch) {
        return borderPointAtRelativeAngle(yaw, pitch).distance(Point3D.ZERO);
    }

    @Override
    public Point3D[] vertexAt(Pose pose) {
        Point3D[] corners_rotated = new Point3D[CORNERS.length];
        for(int i = 0; i < CORNERS.length; i++){
            corners_rotated[i] = CORNERS[i].rotate(pose.getYaw(), pose.getPitch(), pose.getRoll());
        }
        return corners_rotated;
    }

    @Override
    public Vector3D[] axisAt(Pose pose) {
        return new Vector3D[]{
            AXIS_X.rotate(pose.getYaw(), pose.getPitch(), pose.getRoll()),
            AXIS_Y.rotate(pose.getYaw(), pose.getPitch(), pose.getRoll()),
            AXIS_Z.rotate(pose.getYaw(), pose.getPitch(), pose.getRoll())
        };
    }

    @Override
    public float getMinRadius() {
        return minRadius;
    }

    @Override
    public float getMaxRadius() {
        return maxRadius;
    }

    @Override
    protected void loadConfig(HierarchicalConfiguration config) {
        this.halfDimX = config.getFloat(SUBID_DIMX, Float.NaN) / 2;
        if(Float.isNaN(halfDimX)){
            throw new RuntimeException("required field " + SUBID_DIMX + " is empty");
        }
        this.halfDimY = config.getFloat(SUBID_DIMY, Float.NaN) / 2;
        if(Float.isNaN(halfDimY)){
            throw new RuntimeException("required field " + SUBID_DIMY + " is empty");
        }
        this.halfDimZ = config.getFloat(SUBID_DIMZ, Float.NaN) / 2;
        if(Float.isNaN(halfDimZ)){
            throw new RuntimeException("required field " + SUBID_DIMZ + " is empty");
        }
    }
}

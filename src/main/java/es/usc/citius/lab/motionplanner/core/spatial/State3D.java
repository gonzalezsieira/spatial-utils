package es.usc.citius.lab.motionplanner.core.spatial;

import es.usc.citius.lab.motionplanner.core.util.MathFunctions;
import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;

import javax.swing.plaf.nimbus.State;
import java.io.Serializable;

/**
 * 3D state with angles and velocities in X, Y, Z
 *
 * @since 03/10/2017
 */
public class State3D extends Pose3D implements Serializable{

    public static final State3D ZERO = new State3D(Pose3D.ZERO, 0f, 0f, 0f, 0f, 0f, 0f);
    private static final long serialVersionUID = 20171003L;

    float vx;
    float vy;
    float vz;
    float vyaw;
    float vpitch;
    float vroll;

    public State3D(float x, float y, float z, float yaw, float pitch, float roll, float vx, float vy, float vz, float vyaw, float vpitch, float vroll) {
        super(x, y, z, yaw, pitch, roll);
        this.vx = vx;
        this.vy = vy;
        this.vz = vz;
        this.vyaw = vyaw;
        this.vpitch = vpitch;
        this.vroll = vroll;
    }

    public State3D(Pose3D pose, float vx, float vy, float vz, float vyaw, float vpitch, float vroll){
        super(pose);
        this.vx = vx;
        this.vy = vy;
        this.vz = vz;
        this.vyaw = vyaw;
        this.vpitch = vpitch;
        this.vroll = vroll;
    }

    public State3D(State3D other){
        this(other.x, other.y, other.z, other.yaw, other.pitch, other.roll, other.vx, other.vy, other.vz, other.vyaw, other.vpitch, other.vroll);
    }

    /**
     * Builds a {@link SimpleMatrix} with the state values: (x, y, z, yaw, pitch, roll, vx, vy, vz, vyaw, vpitch, vroll )
     *
     * @return a 6x1 {@link SimpleMatrix}
     */
    @Override
    public DenseMatrix64F getMatrix() {
        return new DenseMatrix64F(new double[][]{{x}, {y}, {z}, {yaw}, {pitch}, {roll}, {vx}, {vy}, {vz}, {vyaw}, {vpitch}, {vroll}});
    }


    /************************************************************************
     *                      	GETTERS
     ************************************************************************/
    public float getVx() {
        return vx;
    }

    public float getVy() {
        return vy;
    }

    public float getVz() {
        return vz;
    }

    public float getVyaw() {
        return vyaw;
    }

    public float getVpitch() {
        return vpitch;
    }

    public float getVroll() {
        return vroll;
    }

    /************************************************************************
     * 					GEOMETRIC OPERATION METHODS
     ************************************************************************/

    /**
     * Obtains the symmetric state respect to the X axis:
     * (x, -y, z, reflectedYawX, pitch, reflectedRoll)
     *
     * @return reflected {@link State2D}, respect to the X axis
     */
    @Override
    public State3D symmetricAxisXZ(){
        return new State3D(x, -y, z, MathFunctions.adjustAngleP(-this.yaw), pitch, MathFunctions.adjustAngleP(-this.roll), vx, -vy, vz, -vyaw, vpitch, -vroll);
    }

    /**
     * Obtains the symmetric state respect to the Y axis:
     * (-x, y, z, reflectedYawY, pitch, roll)
     *
     * @return reflected {@link State2D}, respect to the Y axis
     */
    @Override
    public State3D symmetricAxisYZ(){
        return new State3D(-x, y, z, MathFunctions.adjustAngleP(MathFunctions.PI - yaw), pitch, roll, vx, -vy, vz, -vyaw, vpitch, -vroll);
    }

    /**
     * Obtains the symmetric state respect to the Y axis:
     * (x, y, -z, yaw, reflectedPitch, reflectedRoll)
     *
     * @return reflected {@link State2D}, respect to the Y axis
     */
    @Override
    public State3D symmetricAxisXY(){
        return new State3D(x, y, -z, yaw, -pitch, MathFunctions.adjustAngleP(-this.roll), vx, vy, -vz, vyaw, -vpitch, vroll);
    }

    /************************************************************************
     *                 	STATIC GEOMETRIC OPERATION METHODS
     ************************************************************************/
    /**
     * Obtains the state adding the X, Y, Z values of the state and the given point.
     *
     * @param origin origin state, which provides the velocities and orientation
     * @param move coordinates to add
     * @return state resulting of the addition operation
     */
    public static State3D add(State3D origin, Point3D move){
        return new State3D(origin.x + move.x, origin.y + move.y, origin.z + move.z, origin.yaw, origin.pitch, origin.roll, origin.vx, origin.vy, origin.vz, origin.vyaw, origin.vpitch, origin.vroll);
    }

    /**
     * Obtains the state subtracting the X, Y, Z values of the state and the given point.
     *
     * @param origin origin state, which provides the velocities and orientation
     * @param move coordinates to subtract
     * @return state resulting of the subtraction operation
     */
    public static State3D subtract(State3D origin, Point3D move){
        return new State3D(origin.x - move.x, origin.y - move.y, origin.z - move.z, origin.yaw, origin.pitch, origin.roll, origin.vx, origin.vy, origin.vz, origin.vyaw, origin.vpitch, origin.vroll);
    }

    /**
     * Rotates the position and heading of the state keeping the velocities.
     *
     * @param origin origin state, providing the velocities
     * @param roll rotation in roll
     * @param pitch rotation in pitch
     * @param yaw rotation in yaw
     * @return rotated state (position and heading) keeping the velocities as they are in local frame
     */
    public static State3D rotate(State3D origin, float roll, float pitch, float yaw){

        //rotate coordinates
        float[] rotatedXYZ = Point3D.rotateXYZCoordinates(origin.x, origin.y, origin.z, yaw, pitch, roll);

        //rotated heading
        float newYaw = MathFunctions.adjustAngleP(origin.yaw + yaw);
        float newPitch = Math.abs(MathFunctions.adjustAngleP(origin.pitch + pitch));
        float newRoll = MathFunctions.adjustAngleP(origin.roll + roll);

        //keep velocities
        return new State3D(rotatedXYZ[0], rotatedXYZ[1], rotatedXYZ[2], newYaw, newPitch, newRoll, origin.vx, origin.vy, origin.vz, origin.vyaw, origin.vpitch, origin.vroll);

    }
}

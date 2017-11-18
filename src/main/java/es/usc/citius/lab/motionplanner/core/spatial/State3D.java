package es.usc.citius.lab.motionplanner.core.spatial;

import es.usc.citius.lab.motionplanner.core.util.MathFunctions;
import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;

import java.io.Serializable;

/**
 * 3D state with angles and velocities in X, Y, Z
 *
 * @since 03/10/2017
 */
public class State3D extends Pose3D implements Serializable, State {

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
    public State3D symmetricPlaneXZ(){
        return new State3D(x, -y, z, MathFunctions.adjustAngleP(-this.yaw), pitch, MathFunctions.adjustAngleP(-this.roll), vx, -vy, vz, -vyaw, vpitch, -vroll);
    }

    /**
     * Obtains the symmetric state respect to the Y axis:
     * (-x, y, z, reflectedYawY, pitch, roll)
     *
     * @return reflected {@link State2D}, respect to the Y axis
     */
    @Override
    public State3D symmetricPlaneYZ(){
        return new State3D(-x, y, z, MathFunctions.adjustAngleP(MathFunctions.PI - yaw), pitch, roll, vx, -vy, vz, -vyaw, vpitch, -vroll);
    }

    /**
     * Obtains the symmetric state respect to the Y axis:
     * (x, y, -z, yaw, reflectedPitch, reflectedRoll)
     *
     * @return reflected {@link State2D}, respect to the Y axis
     */
    @Override
    public State3D symmetricPlaneXY(){
        return new State3D(x, y, -z, yaw, -pitch, MathFunctions.adjustAngleP(-this.roll), vx, vy, -vz, vyaw, -vpitch, vroll);
    }

    /************************************************************************
     *                 	STATIC GEOMETRIC OPERATION METHODS
     ************************************************************************/
    /**
     * Obtains the state adding the X, Y, Z values of the state and the given point.
     *
     * @param move coordinates to add
     * @return state resulting of the addition operation
     */
    @Override
    public State3D add(Point move){
        return new State3D(this.x + move.getX(), this.y + move.getY(), this.z + move.getZ(), this.yaw, this.pitch, this.roll, this.vx, this.vy, this.vz, this.vyaw, this.vpitch, this.vroll);
    }

    /**
     * Obtains the state subtracting the X, Y, Z values of the state and the given point.
     *
     * @param move coordinates to subtract
     * @return state resulting of the subtraction operation
     */
    @Override
    public State3D subtract(Point move){
        return new State3D(this.x - move.getX(), this.y - move.getY(), this.z - move.getZ(), this.yaw, this.pitch, this.roll, this.vx, this.vy, this.vz, this.vyaw, this.vpitch, this.vroll);
    }

    /**
     * Rotates the position and heading of the state keeping the velocities.
     *
     * @param roll rotation in roll
     * @param pitch rotation in pitch
     * @param yaw rotation in yaw
     * @return rotated state (position and heading) keeping the velocities as they are in local frame
     */
    public State3D rotate(float yaw, float pitch, float roll){
        float[][] rotation = Pose3D.rotateXYZCoordinates(this.x, this.y, this.z, this.yaw, this.pitch, this.roll, yaw, pitch, roll);
        //keep velocities
        return new State3D(rotation[0][0], rotation[0][1], rotation[0][2], rotation[1][0], rotation[1][1], rotation[1][2], this.vx, this.vy, this.vz, this.vyaw, this.vpitch, this.vroll);
    }

    @Override
    public State3D clone() {
        return new State3D(this);
    }

    @Override
    public int symmetryPlane() {
        //obtain angle / (pi / 2) mod 2. If angleSymmetriAxis == 0 then Axis = X, else Axis = Y
        return Math.abs(Math.round( 2 * this.yawTo(Point3D.ZERO) / MathFunctions.PI)) % 2;
    }

    @Override
    public String toString() {
        return "([x=" + x + ", y=" + y + ", z=" + z + ", yaw=" + yaw + "], vx=" + vx + ", vy=" + vy + ", vz=" + vz + ", vyaw=" + vyaw + ")";
    }
}

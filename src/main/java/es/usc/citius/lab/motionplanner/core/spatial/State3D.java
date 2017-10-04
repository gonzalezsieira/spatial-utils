package es.usc.citius.lab.motionplanner.core.spatial;

import javax.swing.plaf.nimbus.State;
import java.io.Serializable;

/**
 * 3D state with angles and velocities in X, Y, Z
 *
 * @since 03/10/2017
 */
public class State3D extends Pose3D implements Serializable{

    public static final State3D ZERO = new State3D(Pose3D.ZERO, 0,0, 0);
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
}

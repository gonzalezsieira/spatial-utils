package es.usc.citius.lab.motionplanner.core.util;

import org.apache.commons.math3.util.FastMath;
import org.ejml.data.FixedMatrix3x3_64F;

public class RotationUtils {

    /**
     * Builds a rotation matrix with determined by three angles: yaw, pitch. roll.
     * @see <a href="http://planning.cs.uiuc.edu/node102.html</a>
     * @param yaw rotation around Z
     * @param pitch rotation around Y
     * @param roll rotation around X
     * @return
     */
    public static FixedMatrix3x3_64F rotationMatrix(float yaw, float pitch, float roll){
        FixedMatrix3x3_64F rotation = new FixedMatrix3x3_64F();

        //pre-calculate sin/cos
        double sinYaw = FastMath.sin(yaw);
        double cosYaw = FastMath.cos(yaw);
        double sinPitch = FastMath.sin(pitch);
        double cosPitch = FastMath.cos(pitch);
        double sinRoll = FastMath.sin(roll);
        double cosRoll = FastMath.cos(roll);

        //first row
        rotation.a11 = cosYaw * cosPitch;
        rotation.a12 = cosYaw * sinPitch * sinRoll - sinYaw * cosRoll;
        rotation.a13 = cosYaw * sinPitch * cosRoll + sinYaw * sinRoll;
        //second row
        rotation.a21 = sinYaw * cosPitch;
        rotation.a22 = sinYaw * sinPitch * sinRoll + cosYaw * cosRoll;
        rotation.a23 = sinYaw * sinPitch * cosRoll - cosYaw * sinRoll;
        //third row
        rotation.a31 = - sinPitch;
        rotation.a32 = cosPitch * sinRoll;
        rotation.a33 = cosPitch * cosRoll;

        //result
        return rotation;
    }

}

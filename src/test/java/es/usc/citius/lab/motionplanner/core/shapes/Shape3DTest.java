package es.usc.citius.lab.motionplanner.core.shapes;

import es.usc.citius.lab.motionplanner.core.spatial.Point3D;
import org.junit.Test;

import java.util.Arrays;
import java.util.Random;

public class Shape3DTest {

    @Test
    public void testPointsInBorder(){
        float[] x = new float[1000];
        float[] y = new float[1000];
        float[] z = new float[1000];
        Shape3D shape = new ShapeRectangle3D(2, 4, 6);
        //initialize random
        Random random = new Random(System.currentTimeMillis());
        //generate random yaw, pitch
        for(int i = 0; i < 1000; i++){
            float yaw = (random.nextFloat() - 0.5f) * 2 * (float) Math.PI;
            float pitch = (random.nextFloat() - 0.5f) * 2 * (float) Math.PI / 2;
            Point3D border = shape.borderPointAtRelativeAngle(yaw, pitch);
            x[i] = border.getX();
            y[i] = border.getY();
            z[i] = border.getZ();
        }

        System.out.println("x = " + Arrays.toString(x));
        System.out.println("y = " + Arrays.toString(y));
        System.out.println("z = " + Arrays.toString(z));

    }

}

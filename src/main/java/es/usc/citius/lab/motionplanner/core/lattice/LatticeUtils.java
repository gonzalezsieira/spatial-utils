package es.usc.citius.lab.motionplanner.core.lattice;

import es.usc.citius.lab.motionplanner.core.spatial.Point;
import es.usc.citius.lab.motionplanner.core.spatial.Point3D;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

public class LatticeUtils {

    /**
     * This method generates a list of neighbors within a given level of neighborhood.
     *
     * @param level desired level of neighborhood (distance in n-steps)
     * @param rX resolution in X
     * @param rY resolution in Y
     * @return list of points withing n-steps of neighborhood from (0,0,0)
     */
    public static List<Point> generateGridPoints(int level, float rX, float rY, float rZ) {
        HashSet<Point> positions = new HashSet<Point>();
        for (int x = -level; x <= level; x++) {
            for (int y = -level; y <= level; y++) {
                for (int z = -level; z <= level; z++) {
                    if (
                            (Math.abs(rX) > 0 && (x == -level || x == level)) ||
                                    (Math.abs(rY) > 0 && (y == -level || y == level)) ||
                                    (Math.abs(rZ) > 0 && (z == -level || z == level))) {
                        positions.add(new Point3D(rX * x, rY * y, rZ * z));
                    }
                }
            }
        }
        return new ArrayList<Point>(positions);
    }

}

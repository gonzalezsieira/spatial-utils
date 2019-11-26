package es.usc.citius.lab.motionplanner.core.lattice;

import es.usc.citius.lab.motionplanner.core.spatial.Point;
import es.usc.citius.lab.motionplanner.core.spatial.Point2D;
import org.apache.commons.math3.util.FastMath;

import java.util.Arrays;
import java.util.Collection;

/**
 * This class adapts the position instance to a defined number of decimals of
 * precision, to avoid the errors of the nodes that are derived of the non standard
 * identifiers of the nodes.
 *
 * @author Adrián González Sieira <<a href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>>
 * @since 28/03/2011
 */
public class PointAdapter2D implements ResolutionAdapter<Point> {

    private float dx;
    private float dy;

    /**
     * This method instantiates a new position adapter with the defined
     * parameters.
     *
     * @param dx difference between positions X
     * @param dy difference between positions Y
     */
    public PointAdapter2D(float dx, float dy) {
        this.dx = dx;
        this.dy = dy;
    }

    /**
     * This method adapts the identifier of the node, taking in consideration
     * the type of position that is defined in the instance of the identifier
     * passed as a parameters.
     *
     * @param id position instance to be adapted
     * @return the adapted position instance with the precision specified
     */
    @Override
    public Point2D adaptNodeID(Point id) {
        float xAdapted = FastMath.round(id.getX() / this.dx) * this.dx;
        float YAdapted = FastMath.round(id.getY() / this.dy) * this.dy;
        float pX = FastMath.round(xAdapted * PRECISION) / PRECISION;
        float pY = FastMath.round(YAdapted * PRECISION) / PRECISION;
        return new Point2D(pX, pY);
    }

    @Override
    public Collection<Point> getIDFromPosition(float x, float y, float z) {
        return Arrays.asList((Point) new Point2D(x, y));
    }
}

package es.usc.citius.lab.motionplanner.core.lattice;

import java.util.Collection;

/**
 * This interface is created to define the node adapter, to avoid
 * the errors referent to the precision of the identifiers of
 * the nodes.
 *
 * @author Adrián González Sieira <<a href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>>
 * @since 22/03/2013
 *
 * @param <T> object type
 */
public interface ResolutionAdapter<T> {

    public static final float PRECISION = 1000;

    /**
     * This method builds a new instance of the identifier, based
     * on the information contained in the original ID.
     *
     * @param idOld original identifier of the node (probably with errors)
     * @return the adapted identifier of the node
     */
    public T adaptNodeID(T idOld);

    /**
     * This method returns the collection of identifiers of the nodes
     * that matches with a position in the map.
     *
     * @param x x coordinate
     * @param y y coordinate
     * @param z z coordinate
     * @return the collection of identifiers in that position
     */
    public Collection<T> getIDFromPosition(float x, float y, float z);

}

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
package es.usc.citius.lab.motionplanner.core.shapes;

import org.apache.commons.configuration.HierarchicalConfiguration;

import java.io.Serializable;
import java.lang.reflect.InvocationTargetException;

/**
 *
 *
 * @author Adrián González Sieira <<a href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>>
 */
public abstract class Shape implements Serializable {

    protected static final String SUBID_CLASS = "class";
    protected static final String SUBID_PARAM = "parameters";
    private static final long serialVersionUID = 20170822L;


    /**
     * Retrieves the optimistic radius of the robot shape.
     *
     * @return
     */
    public abstract float getMinRadius();

    /**
     * Retrieves the pessimistic radius of the robot shape.
     *
     * @return
     */
    public abstract float getMaxRadius();

    /**
     * Loads the information of the shape contained in a
     * {@link HierarchicalConfiguration} file.
     *
     * @param config
     */
    protected abstract void loadConfig(HierarchicalConfiguration config);

    /**
     * Obtains a new instance based on the configuration passed as an
     * argument.
     *
     * @param config {@code <shape>...</shape>} configuration group
     * @return instance of {@link Shape} with the parameters specified
     */
    public static Shape create(HierarchicalConfiguration config){
        //retrieve class of the shape
        String className = config.getString(Shape.SUBID_CLASS, "");
        if(className.isEmpty()){
            throw new RuntimeException("required value robot.shape.class is missing");
        }
        try{
            return (Shape) ClassLoader.getSystemClassLoader().loadClass(className).getDeclaredConstructor(HierarchicalConfiguration.class).newInstance(config);
        } catch(ClassNotFoundException ex) {
            throw new RuntimeException("class " + className + " cannot be found: " + ex);
        } catch(NoSuchMethodException ex) {
            throw new RuntimeException("referenced class " + className + " does not implement new(HierarchicalConfiguration): " + ex);
        } catch(InstantiationException ex){
            throw new RuntimeException("referenced class " + className + " is not instantiable: " + ex);
        } catch(IllegalAccessException ex){
            throw new RuntimeException("constructor new(HierarchicalConfiguration) of the class " + className + " is not accesible: " + ex);
        } catch(InvocationTargetException ex){
            throw new RuntimeException("Internal error in the constructor of the class: " + className + "; " + ex.getTargetException().toString());
        }
    }
}

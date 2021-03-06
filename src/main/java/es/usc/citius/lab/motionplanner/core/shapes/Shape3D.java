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

import es.usc.citius.lab.motionplanner.core.spatial.*;
import org.apache.commons.configuration.HierarchicalConfiguration;

/**
 *
 *
 * @author Adrián González Sieira <<a href="mailto:adrian.gonzalez@usc.es">adrian.gonzalez@usc.es</a>>
 */
public abstract class Shape3D extends Shape {

    protected static final String SUBID_CLASS = "class";
    protected static final String SUBID_PARAM = "parameters";
    private static final long serialVersionUID = 20170823L;

    /**
     * Empty constructor.
     */
    protected Shape3D(){
        //empty constructor
    }

    /**
     * Initializes the shape from a {@link HierarchicalConfiguration} file.
     *
     * @param config configuration
     */
    protected Shape3D(HierarchicalConfiguration config){
        loadConfig(config);
    }

    @Override
    public abstract Point3D borderPointAtRelativeAngle(float yaw, float pitch);

    @Override
    public abstract Point3D[] vertexAt(Pose pose);

    @Override
    public abstract Vector3D[] axisAt(Pose pose);

}

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

import java.util.List;
import org.apache.commons.configuration.ConfigurationException;
import org.apache.commons.configuration.HierarchicalConfiguration;
import org.apache.commons.configuration.XMLConfiguration;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Tests the generic methods of {@link Shape2D}.
 *
 * @author Adrián González Sieira <a href=mailto:adrian.gonzalez@usc.es>adrian.gonzalez@usc.es</a>
 */
public class Shape2DTest {
    
    private static final String FILE = "/config/test.xml";
    private static final String SHAPE = "shape";
    private XMLConfiguration config;
    private List<HierarchicalConfiguration> shapesConfig;
    
    @Before
    public void setUp() throws ConfigurationException{
        this.config = new XMLConfiguration(getClass().getResource(FILE));
        this.shapesConfig = config.configurationsAt(SHAPE);
    }

    /**
     * Checks the root node of the HierarchicalConfiguration.
     */
    @Test
    public void test_createFromXML() {
        for(HierarchicalConfiguration current : shapesConfig){
            //test the root label of the configuration
            assertEquals("Root element name of shapes config must be <shape>", SHAPE, current.getRoot().getName());
        }
    }
    
    /**
     * Checks the type of the shape and the expected one.
     */
    @Test
    public void test_class() {
        for(HierarchicalConfiguration current : shapesConfig){
            //instantiate class
            Shape shape = Shape.create(current);
            //compare expected and current class names
            String className = shape.getClass().getCanonicalName();
            String expectedName = current.getString("class");
            assertEquals("Instantiated class does not match expected", expectedName, className);
        }
    }

}

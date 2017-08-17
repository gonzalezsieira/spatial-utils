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
package es.usc.citius.lab.motionplanner.core.util;

/**
 * Created by adrian.gonzalez on 7/12/16.
 */
public class RandomUtils {

    /**
     * Transforms a value into a given range.
     * @param value
     * @return
     */
    public static float randomToValue(float value, float max){
        float valueUsed = (value - 0.5f);
        int sign = (int) (valueUsed/Math.abs(valueUsed));
        return (float) (Math.min(Math.max(Math.abs(valueUsed), 0.1), 0.4) * 2 * max * sign);
    }

}

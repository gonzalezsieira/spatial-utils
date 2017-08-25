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

import java.io.Serializable;

/**
 * Container for three elements with predefined types.
 *
 * @param <T> type of first element
 * @param <Q> type of second element
 * @param <R> type of third element
 */
public class Triple<T extends Object, Q extends Object, R extends Object> implements Serializable{

    //typed attributes
    private T first;
    private Q second;
    private R third;
    //to serialize class
    private static final long serialVersionUID = 20170825L;

    /**
     * Default constructor with three elements.
     *
     * @param first
     * @param second
     * @param third
     */
    public Triple(T first, Q second, R third) {
        this.first = first;
        this.second = second;
        this.third = third;
    }

    public T getFirst() {
        return first;
    }

    public Q getSecond() {
        return second;
    }

    public R getThird() {
        return third;
    }

    public void setFirst(T first) {
        this.first = first;
    }

    public void setSecond(Q second) {
        this.second = second;
    }

    public void setThird(R third) {
        this.third = third;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Triple<?, ?, ?> triple = (Triple<?, ?, ?>) o;

        if (!first.equals(triple.first)) return false;
        if (!second.equals(triple.second)) return false;
        return third.equals(triple.third);
    }

    @Override
    public int hashCode() {
        int result = first.hashCode();
        result = 31 * result + second.hashCode();
        result = 31 * result + third.hashCode();
        return result;
    }
}

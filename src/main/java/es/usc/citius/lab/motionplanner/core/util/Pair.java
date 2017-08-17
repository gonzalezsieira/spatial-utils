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
 * This class encapsulates a container of a pair of objects, as key and content,
 * that are associated into this structure.
 *
 * @author Adrián González
 */
public class Pair<T extends Object, Q extends Object> implements Serializable {

    /**
	 * Generated serialVersionUID.
	 */
	private static final long serialVersionUID = -764048328330200339L;
	public T key;
    public Q content;

    public Pair(T key, Q content) {
        this.key = key;
        this.content = content;
    }

    public T getKey() {
        return key;
    }

    public Q getContent() {
        return content;
    }

    public void setKey(T key) {
        this.key = key;
    }

    public void setContent(Q content) {
        this.content = content;
    }

    @Override
	public int hashCode() {
		final int prime = 53;
		int result = 3;
		result = prime * result + ((content == null) ? 0 : content.hashCode());
		result = prime * result + ((key == null) ? 0 : key.hashCode());
		return result;
	}

    @Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Pair<?, ?> other = (Pair<?, ?>) obj;
		if (content == null) {
			if (other.content != null)
				return false;
		} else if (!content.equals(other.content))
			return false;
		if (key == null) {
			if (other.key != null)
				return false;
		} else if (!key.equals(other.key))
			return false;
		return true;
	}

	@Override
    public String toString() {
        return "<" + this.key.toString() + " " + this.content.toString() + ">";
    }
}

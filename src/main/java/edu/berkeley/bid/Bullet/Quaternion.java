package edu.berkeley.bid.Bullet;

import java.io.Serializable;

public class Quaternion implements Serializable {
	private static final long serialVersionUID = 5434563258976438528L;

	public float x;
	public float y;
	public float z;
	public float w;

	public Quaternion (float x, float y, float z, float w) {
		this.set(x, y, z, w);
	}

	public Quaternion () {
		idt();
	}

	public Quaternion (Quaternion quaternion) {
		this.set(quaternion);
	}

	public Quaternion set (float x, float y, float z, float w) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.w = w;
		return this;
	}

	public Quaternion set (Quaternion quaternion) {
		return this.set(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
	}

	public Quaternion cpy () {
		return new Quaternion(this);
	}

	public final static float len (final float x, final float y, final float z, final float w) {
		return (float)Math.sqrt(x * x + y * y + z * z + w * w);
	}

	public float len () {
		return (float)Math.sqrt(x * x + y * y + z * z + w * w);
	}

	@Override
	public String toString () {
		return "[" + x + "|" + y + "|" + z + "|" + w + "]";
	}

    	public Quaternion idt () {
		return this.set(0, 0, 0, 1);
	}


}

package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class Vector3 implements Serializable {
    private static final long serialVersionUID = 34937421973128374L;

    public float x;
    public float y;
    public float z;

    public Vector3 () {
    }

    public Vector3 (float x, float y, float z) {
	this.set(x, y, z);
    }

    public Vector3 (final Vector3 vector) {
	this.set(vector);
    }

    public Vector3 (final float[] values) {
	this.set(values[0], values[1], values[2]);
    }

    public Vector3 set (float x, float y, float z) {
	this.x = x;
	this.y = y;
	this.z = z;
	return this;
    }

    public Vector3 set (final Vector3 vector) {
	return this.set(vector.x, vector.y, vector.z);
    }

    public Vector3 set (final float[] values) {
	return this.set(values[0], values[1], values[2]);
    }

    @Override
    public String toString () {
	return "[" + x + "|" + y + "|" + z + "]";
    }

    public Vector3 cpy () {
	return new Vector3(this);
    }
}

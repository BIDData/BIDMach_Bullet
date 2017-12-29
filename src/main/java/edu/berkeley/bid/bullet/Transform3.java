package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class Transform3 implements Serializable {
    private static final long serialVersionUID = 3242197895326574L;

    public Matrix3x3 m_basis;
    public Vector3 m_origin;

    public Transform3 (Matrix3x3 basis, Vector3 origin) {
	m_basis = basis.cpy();
	m_origin = origin.cpy();
    }

    public Transform3() {
	m_basis = new Matrix3x3();
	m_origin = new Vector3();
    }

    public Transform3(Transform3 m) {
	this.set(m);
    }

    public Transform3 set(Matrix3x3 basis, Vector3 origin) {
	m_basis = basis.cpy();
	m_origin = origin.cpy();
	return this;
    }

    public Transform3 set(Transform3 m) {
	this.set(m.m_basis, m.m_origin);
	return this;
    }

    public Transform3 cpy() {
	return new Transform3(this);
    }

    @Override
    public String toString () {
	return "[basis=" + m_basis.toString() + ", origin=" + m_origin.toString()+"]";
    }

}

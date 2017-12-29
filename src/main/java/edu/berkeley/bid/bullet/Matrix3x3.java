package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class Matrix3x3 implements Serializable {
    private static final long serialVersionUID = 8903214321342893L;

    public Vector3 m_el[];

    public Matrix3x3 () {
	m_el = new Vector3[3];
    }

    public Matrix3x3(Vector3 el[]) {
	this.set(el);
    }

    public Matrix3x3(Matrix3x3 m) {
	this.set(m);
    }

    public Matrix3x3 set(Vector3 el[]) {
	m_el[0] = el[0].cpy();
	m_el[1] = el[1].cpy();
	m_el[2] = el[2].cpy();
	return this;
    }

    public Matrix3x3 set(Matrix3x3 m) {
	this.set(m.m_el);
	return this;
    }

    public Matrix3x3 cpy() {
	return new Matrix3x3(this);
    }

    public Matrix3x3 idt() {
	m_el[0].set(1, 0, 0);
	m_el[1].set(0, 1, 0);
	m_el[2].set(0, 0, 1);
	return this;
    }

}

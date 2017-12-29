package edu.berkeley.bid.Bullet;

import java.io.Serializable;

public class JointInfo implements Serializable {
    private static final long serialVersionUID = -2314234211478698963L;

    public String m_linkName;
    public String m_jointName;
    public int m_jointType;
    public int m_qIndex;
    public int m_uIndex;
    public int m_jointIndex;
    public int m_flags;
    public double m_jointDamping;
    public double m_jointFriction;
    public double m_jointLowerLimit;
    public double m_jointUpperLimit;
    public double m_jointMaxForce;
    public double m_jointMaxVelocity;
    public double m_parentFrame[]; // position and orientation (quaternion)
    public double m_childFrame[]; // ^^^
    public double m_jointAxis[]; // joint axis in parent local frame

    public JointInfo (String linkName, String jointName) {
	m_linkName = linkName;
	m_jointName = jointName;
	m_parentFrame = new double[7];
	m_childFrame = new double[7];
	m_jointAxis = new double[3];
    }

    public JointInfo() {
	m_parentFrame = new double[7];
	m_childFrame = new double[7];
	m_jointAxis = new double[3];
    }
}

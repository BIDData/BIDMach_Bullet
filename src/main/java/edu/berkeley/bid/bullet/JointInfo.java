package edu.berkeley.bid.bullet;

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
    public double []m_parentFrame; // position and orientation (quaternion)
    public double []m_childFrame; // ^^^
    public double []m_jointAxis; // joint axis in parent local frame

    public JointInfo (String linkName, String jointName) {
	m_linkName = linkName;
	m_jointName = jointName;
	m_parentFrame = new double[7];
	m_childFrame = new double[7];
	m_jointAxis = new double[3];
    }

    public JointInfo (String linkName, String jointName, int jointType, int qIndex, int uIndex, int jointIndex, int flags,
		      double jointDamping, double jointFriction, double jointLowerLimit, double jointUpperLimit,
		      double jointMaxForce, double jointMaxVelocity, double [] parentFrame, double [] childFrame, double [] jointAxis) {
	m_linkName = linkName;
	m_jointName = jointName;
	m_jointType = jointType;
	m_qIndex = qIndex;
	m_uIndex = uIndex;
	m_jointIndex = jointIndex;
	m_flags = flags;
	m_jointDamping = jointDamping;
	m_jointFriction = jointFriction;
	m_jointLowerLimit = jointLowerLimit;
	m_jointUpperLimit = jointUpperLimit;
	m_jointMaxForce = jointMaxForce;
	m_jointMaxVelocity = jointMaxVelocity;
	m_parentFrame = parentFrame;
	m_childFrame = childFrame;
	m_jointAxis = jointAxis;
    }

    public JointInfo() {
	m_parentFrame = new double[7];
	m_childFrame = new double[7];
	m_jointAxis = new double[3];
    }

    @Override
    public String toString () {
	return "[" +
	    "jointIndex=" + m_jointIndex + ", " +
	    "jointName=" + m_jointName + ", " +
	    "jointType=" + m_jointType + ", " +
	    "qIndex=" + m_qIndex + ", " +
	    "uIndex=" + m_uIndex + ", " +
	    "flags=" + m_flags + ", " +
	    "jointDamping=" + m_jointDamping + ", " +
	    "jointFriction=" + m_jointFriction + ", " +
	    "jointLowerLimit=" + m_jointLowerLimit + ", " +
	    "jointUpperLimit=" + m_jointUpperLimit + ", " +
	    "jointMaxForce=" + m_jointMaxForce + ", " +
	    "jointMaxVelocity=" + m_jointMaxVelocity + ", " +
	    "linkName=" + m_linkName + ", " +
	    "parentFrame=" + m_parentFrame + ", " +
	    "childFrame=" + m_childFrame + ", " +
	    "jointAxis=" + m_jointAxis + "]";
    }

}

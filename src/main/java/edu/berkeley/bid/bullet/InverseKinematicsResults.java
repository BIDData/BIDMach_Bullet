package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class JointMotorArgs implements Serializable {
    private static final long serialVersionUID = 543546378546385L;

    public int m_controlMode;

    public double m_targetPosition;
    public double m_kp;

    public double m_targetVelocity;
    public double m_kd;

    public double m_maxTorqueValue;

    public JointMotorArgs(int controlMode) {
	m_controlMode = controlMode;
	m_targetPosition = 0;
	m_kp = 0.1;
	m_targetVelocity = 0;
	m_kd = 0.9;
	m_maxTorqueValue = 1000;
    }

    public JointMotorArgs(int controlMode, double targetPosition, double kp, double targetVelocity, double kd, double maxTorqueValue) {
	m_controlMode = controlMode;
	m_targetPosition = targetPosition;
	m_kp = kp;
	m_targetVelocity = targetVelocity;
	m_kd = kd;
	m_maxTorqueValue = maxTorqueValue;
    }

    @Override
    public String toString () {
	return "[" +
	    "controlMode=" + m_controlMode + ", " +
	    "targetPosition=" + m_targetPosition + ", " +
	    "kp=" + m_kp + ", " +
	    "targetVelocity=" + m_targetVelocity + ", " +
	    "kd=" + m_kd + ", " +
	    "maxTorqueValue=" + m_maxTorqueValue + "]";
    }

}

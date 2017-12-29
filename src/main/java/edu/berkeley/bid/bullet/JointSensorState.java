package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class JointSensorState implements Serializable {
    private static final long serialVersionUID = 32106451303123498L;

    public double m_jointPosition;
    public double m_jointVelocity;
    public double []m_jointForceTorque;
    public double m_jointMotorTorque;

    public JointSensorState (double jointPosition, double jointVelocity, double []jointForceTorque, double jointMotorTorque) {
	m_jointPosition = jointPosition;
	m_jointVelocity = jointVelocity;
	m_jointForceTorque = jointForceTorque;
	m_jointMotorTorque = jointMotorTorque;
    }

    public JointSensorState() {
	m_jointForceTorque = new double[6];
    }

    @Override
    public String toString () {
    	return "[" +
	    "jointPosition=" + m_jointPosition + ", " +
	    "jointVelocity=" + m_jointVelocity + ", " +
	    "jointForceTorque=" + m_jointForceTorque + ", " +
	    "jointMotorTorque=" + m_jointMotorTorque + "]";
    }

}

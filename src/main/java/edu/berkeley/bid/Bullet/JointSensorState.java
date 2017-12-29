package edu.berkeley.bid.Bullet;

import java.io.Serializable;

public class JointSensorState implements Serializable {
    private static final long serialVersionUID = 32106451303123498L;

    double m_jointPosition;
    double m_jointVelocity;
    double m_jointForceTorque[];
    double m_jointMotorTorque;

    public JointSensorState (double jointPosition, double jointVelocity, double jointForceTorque[], double jointMotorTorque) {
	m_jointPosition = jointPosition;
	m_jointVelocity = jointVelocity;
	m_jointForceTorque = jointForceTorque;
	m_jointMotorTorque = jointMotorTorque;
    }

    public JointSensorState() {
	m_jointForceTorque = new double[6];
    }
}

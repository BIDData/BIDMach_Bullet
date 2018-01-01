package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class DynamicsInfo implements Serializable {
    private static final long serialVersionUID = 342183472184927723L;

    double m_mass;
    double [] m_localInertialPosition;
    double m_lateralFrictionCoeff;

    public DynamicsInfo() {
	m_localInertialPosition = new double[3];
    }

    public DynamicsInfo(double mass, double [] localInertialPosition, double lateralFrictionCoeff) {
	m_mass = mass;
	m_lateralFrictionCoeff = lateralFrictionCoeff;
	m_localInertialPosition = new double[3];
	m_localInertialPosition[0] = localInertialPosition[0];
	m_localInertialPosition[1] = localInertialPosition[1];
	m_localInertialPosition[2] = localInertialPosition[2];
    }

    @Override
    public String toString () {
	return "DynamicsInfo[mass=" + m_mass +
	    ",lateralFrictionCoeff=" + m_lateralFrictionCoeff +
	    ",localInertialPosition=[" +
	    m_localInertialPosition[0] + "," +
	    m_localInertialPosition[1] + "," +
	    m_localInertialPosition[2] + "]]";
    }

}

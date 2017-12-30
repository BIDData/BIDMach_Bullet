package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class LinkState implements Serializable {
    private static final long serialVersionUID = 4783245849535423L;

    double [] m_worldPosition;
    double [] m_worldOrientation;

    double [] m_localInertialPosition;
    double [] m_localInertialOrientation;

    double [] m_worldLinkFramePosition;
    double [] m_worldLinkFrameOrientation;

    double [] m_worldLinearVelocity; 
    double [] m_worldAngularVelocity;

    double [] m_worldAABBMin;
    double [] m_worldAABBMax;

    public LinkState() {
	m_worldPosition = new double[3];
	m_worldOrientation = new double[4];
	m_worldOrientation[3] = 1;
	m_localInertialPosition = new double[3];
	m_localInertialOrientation = new double[4];
	m_localInertialOrientation[3] = 1;
	m_worldLinkFramePosition = new double[3];
	m_worldLinkFrameOrientation = new double[4];
	m_worldLinkFrameOrientation[3] = 1;
	m_worldLinearVelocity = new double[3];
	m_worldAngularVelocity = new double[3];
	m_worldAABBMin = new double[3];
	m_worldAABBMax = new double[3];
    }

    public LinkState(double [] worldPosition, double [] worldOrientation, double [] localInertialPosition, double [] localInertialOrientation,
		     double [] worldLinkFramePosition, double [] worldLinkFrameOrientation, double [] worldLinearVelocity, double [] worldAngularVelocity,
		     double [] worldAABBMin, double [] worldAABBMax) {
	m_worldPosition = worldPosition;
	m_worldOrientation = worldOrientation;
	m_localInertialPosition = localInertialPosition;
	m_localInertialOrientation = localInertialOrientation;
	m_worldLinkFramePosition = worldLinkFramePosition;
	m_worldLinkFrameOrientation = worldLinkFrameOrientation;
	m_worldLinearVelocity = worldLinearVelocity;
	m_worldAngularVelocity = worldAngularVelocity;
	m_worldAABBMin = worldAABBMin;
	m_worldAABBMax = worldAABBMax;
    }


    @Override
    public String toString () {
	return "[" +
	    "worldPosition=(" + m_worldPosition[0] + "," + m_worldPosition[1] + "," + m_worldPosition[2] + ")," +
	    "worldOrientation=(" + m_worldOrientation[0] + "," + m_worldOrientation[1] + "," + m_worldOrientation[2] + "," + m_worldOrientation[3] + ")," +
	    "localInertialPosition=(" + m_localInertialPosition[0] + "," + m_localInertialPosition[1] + "," + m_localInertialPosition[2] + ")," +
	    "localInertialOrientation=(" + m_localInertialOrientation[0] + "," + m_localInertialOrientation[1] + "," + m_localInertialOrientation[2] + "," + m_localInertialOrientation[3] + ")," +
	    "worldLinkFramePosition=(" + m_worldLinkFramePosition[0] + "," + m_worldLinkFramePosition[1] + "," + m_worldLinkFramePosition[2] + ")," +
	    "worldLinkFrameOrientation=(" + m_worldLinkFrameOrientation[0] + "," + m_worldLinkFrameOrientation[1] + "," + m_worldLinkFrameOrientation[2] + "," + m_worldLinkFrameOrientation[3] + ")," +
	    "worldLinearVelocity=(" + m_worldLinearVelocity[0] + "," + m_worldLinearVelocity[1] + "," + m_worldLinearVelocity[2] + ")," +
	    "worldAngularVelocity=(" + m_worldAngularVelocity[0] + "," + m_worldAngularVelocity[1] + "," + m_worldAngularVelocity[2] + ")," +
	    "worldAABBMin=(" + m_worldAABBMin[0] + "," + m_worldAABBMin[1] + "," + m_worldAABBMin[2] + ")," +
	    "worldAABBMax=(" + m_worldAABBMax[0] + "," + m_worldAABBMax[1] + "," + m_worldAABBMax[2] + ")]";

    }

}

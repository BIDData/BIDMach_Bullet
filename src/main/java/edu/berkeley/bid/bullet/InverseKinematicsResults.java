package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class InverseKinematicsResults implements Serializable {
    private static final long serialVersionUID = -4532452346789679835L;

    int m_bodyUniqueId;
    double [] m_calculatedJointPositions;

    public InverseKinematicsResults() {
    }

    @Override
    public String toString () {
	return "[" +
	    "bodyUniqueId=" + m_bodyUniqueId + ", " +
	    "calculatedJointPositions=" + m_calculatedJointPositions + "]";
    }

}

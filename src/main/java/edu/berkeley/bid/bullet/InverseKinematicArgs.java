package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class InverseKinematicArgs implements Serializable {
    private static final long serialVersionUID = -12313217678543L;

    public int m_bodyUniqueId;
    public double [] m_endEffectorTargetPosition;
    public double [] m_endEffectorTargetOrientation;
    public int m_endEffectorLinkIndex;
    public int m_flags;
    public int m_numDegreeOfFreedom;
    public double [] m_lowerLimits;
    public double [] m_upperLimits;
    public double [] m_jointRanges;
    public double [] m_restPoses;
    public double [] m_jointDamping;

    public InverseKinematicArgs() {
	m_endEffectorTargetPosition = new double[3];
	m_endEffectorTargetOrientation = new double[4];
	m_bodyUniqueId = -1;
	m_endEffectorLinkIndex = -1;
	m_flags = 0;

	m_endEffectorTargetPosition[0] = 0;
	m_endEffectorTargetPosition[1] = 0;
	m_endEffectorTargetPosition[2] = 0;
	
	m_endEffectorTargetOrientation[0] = 0;
	m_endEffectorTargetOrientation[1] = 0;
	m_endEffectorTargetOrientation[2] = 0;
	m_endEffectorTargetOrientation[3] = 1;
    }

    @Override
    public String toString () {
	return "[" +
	    "bodyUniqueId=" + m_bodyUniqueId + ", " +
	    "endEffectorTargetPosition=" + m_endEffectorTargetPosition + ", " +
	    "endEffectorTargetOrientation=" + m_endEffectorTargetOrientation + ", " +
	    "endEffectorLinkIndex=" + m_endEffectorLinkIndex + ", " +
	    "flags=" + m_flags + ", " +
	    "numDegreeOfFreedom=" + m_numDegreeOfFreedom + ", " +
	    "m_lowerLimits=" + m_lowerLimits + ", " +
	    "m_upperLimits=" + m_upperLimits + ", " +
	    "m_jointRanges=" + m_jointRanges + ", " +
	    "m_restPoses=" + m_restPoses + ", " +
	    "m_jointDamping=" + m_jointDamping + "]";
    }

}

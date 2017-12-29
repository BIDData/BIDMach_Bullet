package edu.berkeley.bid.Bullet;

import java.io.Serializable;

public class JointStates2 implements Serializable {
    private static final long serialVersionUID = -984234234323414323L;

    public int m_bodyUniqueId;
    public int m_numDegreeOfFreedomQ;
    public int m_numDegreeOfFreedomU;
    public Transform3 m_rootLocalInertialFrame;
    public double m_actualStateQ[];
    public double m_actualStateQdot[];
    public double m_jointReactionForces[];

    public JointStates2 (int bodyUniqueId, int numDegreeOfFreedomQ, int numDegreeOfFreedomU, Transform3 rootLocalInertialFrame,
			 double actualStateQ[], double actualStateQdot[], double jointReactionForces[]) {
	m_bodyUniqueId = bodyUniqueId;
	m_numDegreeOfFreedomQ = numDegreeOfFreedomQ;
	m_numDegreeOfFreedomU = numDegreeOfFreedomU;
	m_rootLocalInertialFrame = rootLocalInertialFrame;
	m_actualStateQ = actualStateQ;
	m_actualStateQdot = actualStateQdot;
	m_jointReactionForces = jointReactionForces;
    }

    public JointStates2() {

    }
}

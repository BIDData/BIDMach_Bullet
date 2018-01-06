package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class ContactPointData implements Serializable {
    private static final long serialVersionUID = 7839126482342L;

    public int m_contactFlags;
    public int m_bodyUniqueIdA;
    public int m_bodyUniqueIdB;
    public int m_linkIndexA;
    public int m_linkIndexB;
    public double [] m_positionOnAInWS;//contact point location on object A, in world space coordinates
    public double [] m_positionOnBInWS;//contact point location on object A, in world space coordinates
    public double [] m_contactNormalOnBInWS;//the separating contact normal, pointing from object B towards object A
    public double m_contactDistance;//negative number is penetration, positive is distance.
    
    public double m_normalForce;

    public ContactPointData () {
	m_positionOnAInWS = new double[3];
	m_positionOnBInWS = new double[3];
	m_contactNormalOnBInWS = new double[3]; 
    }

    private String printVector(double [] vector) {
	return "[" + vector[0] + "," + vector[1] + "," + vector[2] + "]";
    }
	    
    @Override
    public String toString () {
    	return "[" +
	    "contactFlags=" + m_contactFlags + "," +
	    "bodyUniqueIdA=" + m_bodyUniqueIdA + "," +
	    "bodyUniqueIdB=" + m_bodyUniqueIdB + "," +
	    "linkIndexA=" + m_linkIndexA + "," +
	    "linkIndexB=" + m_linkIndexB +  "," +
	    "positionOnAInWS=" + printVector(m_positionOnAInWS) + "," +
	    "positionOnBInWS=" + printVector(m_positionOnBInWS) + "," +
	    "contactNormalOnBInWS=" + printVector(m_contactNormalOnBInWS) + "," +
	    "contactDistance=" + m_contactDistance + "]";
    }

}

package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class ContactInformation implements Serializable {
    private static final long serialVersionUID = 4525367823634L;

    public int m_numContactPoints;
    public ContactPointData [] m_contactPointData;

    public ContactInformation() {
    }

    public ContactInformation(int n) {
	m_numContactPoints = n;
	m_contactPointData = new ContactPointData[n];
    }

    @Override
    public String toString () {
    	return "ContactPointInfo[" + m_numContactPoints + "]";
    }

}

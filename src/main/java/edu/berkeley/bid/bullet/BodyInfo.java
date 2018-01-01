package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class BodyInfo implements Serializable {
    private static final long serialVersionUID = 8947527343235449L;

    public String m_baseName;
    public String m_bodyName; 

    public BodyInfo (String baseName, String bodyName) {
	m_baseName = baseName;
	m_bodyName = bodyName;
    }

    public BodyInfo() {
    }

    @Override
    public String toString () {
	return "BodyInfo[baseName=" + m_baseName + ",bodyName=" + m_bodyName + "]";
    }

}

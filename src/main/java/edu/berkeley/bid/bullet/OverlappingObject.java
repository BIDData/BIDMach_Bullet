package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class OverlappingObject implements Serializable {
    private static final long serialVersionUID = -547382932765L;

    public int m_objectUniqueId;
    public int m_linkIndex;

    public OverlappingObject() {
    }

    public OverlappingObject(int objectUniqueId, int linkIndex) {
	m_objectUniqueId = objectUniqueId;
	m_linkIndex = linkIndex;
    }

    @Override
    public String toString () {
	return "[" + m_objectUniqueId + "," + m_linkIndex + "]";
    }

}

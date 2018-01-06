package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class AABBOverlapData implements Serializable {
    private static final long serialVersionUID = 432573427834502L;

    public int m_numOverlappingObjects;
    public OverlappingObject [] m_overlappingObjects;

    public AABBOverlapData() {
    }

    public AABBOverlapData(int n) {
	m_numOverlappingObjects = n;
	m_overlappingObjects = new OverlappingObject[n];
    }

    @Override
    public String toString () {
    	return "AABBOverlapData[" + m_numOverlappingObjects + "]";
    }

}

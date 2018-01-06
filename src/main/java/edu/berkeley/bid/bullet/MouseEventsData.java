package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class MouseEventsData implements Serializable {
    private static final long serialVersionUID = 354289467582653L;

    public int m_numMouseEvents;
    public MouseEvent [] m_mouseEvents;

    public MouseEventsData() {
    }

    public MouseEventsData(int n) {
	m_numMouseEvents = n;
	m_mouseEvents = new MouseEvent[n];
    }

    @Override
    public String toString () {
    	return "MouseEventsData[" + m_numMouseEvents + "]";
    }

}

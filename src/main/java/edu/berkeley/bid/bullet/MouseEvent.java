package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class MouseEvent implements Serializable {
    private static final long serialVersionUID = -543834529362438L;

    public int m_eventType;
    public float m_mousePosX;
    public float m_mousePosY;
    public int m_buttonIndex;
    public int m_buttonState;

    public MouseEvent() {
    }

    public MouseEvent(int eventType, float mouseX, float mouseY, int buttonIndex, int buttonState) {
	m_eventType = eventType;
	m_mousePosX = mouseX;
	m_mousePosY = mouseY;
	m_buttonIndex = buttonIndex;
	m_buttonState = buttonState;
    }

    @Override
    public String toString () {
	return "MouseEvent[" +
	    m_eventType + "," +
	    m_mousePosX + "," +
	    m_mousePosY + "," +
	    m_buttonIndex + "," +
	    m_buttonState + "]";
    }

}

package edu.berkeley.bid.bullet;

import java.io.Serializable;

public class KeyboardEvent implements Serializable {
    private static final long serialVersionUID = -47589324425678L;

    public int m_keyCode;
    public int m_keyState;

    public KeyboardEvent() {
    }

    public KeyboardEvent(int keyCode, int keyState) {
	m_keyCode = keyCode;
	m_keyState = keyState;
    }

    @Override
    public String toString () {
	return "[" + m_keyCode + "," + m_keyState + "]";
    }

}

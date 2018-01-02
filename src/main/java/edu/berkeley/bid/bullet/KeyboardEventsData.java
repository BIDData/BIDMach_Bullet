package edu.berkeley.bid.bullet;

import java.io.Serializable;
import java.lang.StringBuilder;

public class KeyboardEventsData implements Serializable {
    private static final long serialVersionUID = -432167463211342L;

    public int m_numKeyboardEvents;
    public KeyboardEvent [] m_keyboardEvents;

    public KeyboardEventsData() {
	m_numKeyboardEvents = 0;
	m_keyboardEvents = null;
    }

    public KeyboardEventsData(int numKeyboardEvents) {
	m_numKeyboardEvents = numKeyboardEvents;
	if (numKeyboardEvents > 0) {
	    m_keyboardEvents = new KeyboardEvent[numKeyboardEvents];
	    for (int i = 0; i < numKeyboardEvents; i++) {
		m_keyboardEvents[i] = new KeyboardEvent();
	    }
	}
    }

    @Override
    public String toString () {
	StringBuilder sbuilder = new StringBuilder();
	sbuilder.append("[");
	if (m_numKeyboardEvents > 0) {
	    sbuilder.append(m_keyboardEvents[0].toString());
	    for (int i=1; i<m_numKeyboardEvents; i++) {
		sbuilder.append(",");
		sbuilder.append(m_keyboardEvents[i].toString());
	    }
	}
	sbuilder.append("]");
	return sbuilder.toString();
    }

}

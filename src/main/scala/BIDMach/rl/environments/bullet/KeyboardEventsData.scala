package BIDMach.rl.environments.bullet;

@SerialVersionUID(321834718243721L)
class KeyboardEventsData(n:Int) extends edu.berkeley.bid.bullet.KeyboardEventsData(n) {
    def this() = this(0);

    def this(k:edu.berkeley.bid.bullet.KeyboardEventsData) = {
	this(0);
	m_numKeyboardEvents = k.m_numKeyboardEvents;
	m_keyboardEvents = k.m_keyboardEvents;
    };
}

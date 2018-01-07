package BIDMach.rl.environments.bullet;

@SerialVersionUID(-24382346314932617L)
class MouseEventsData(n:Int) extends edu.berkeley.bid.bullet.MouseEventsData(n) {
    def this() = this(0);

    def this(k:edu.berkeley.bid.bullet.MouseEventsData) = {
	this(0);
	m_numMouseEvents = k.m_numMouseEvents;
	m_mouseEvents = k.m_mouseEvents;
    };
}

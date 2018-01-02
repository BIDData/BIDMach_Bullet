package BIDMach.rl.environments.bullet;
import BIDMat.DMat;

@SerialVersionUID(347814368796176834L)
class JointSensorState(jss:edu.berkeley.bid.bullet.JointSensorState) extends Serializable {

    val javaSensorState = jss;
    
    def position:Double = javaSensorState.m_jointPosition;
    def velocity:Double = javaSensorState.m_jointVelocity;
    def motorTorque:Double = javaSensorState.m_jointMotorTorque;

    def position_=(v:Double) = javaSensorState.m_jointPosition = v;
    def velocity_=(v:Double) = javaSensorState.m_jointVelocity = v;
    def motorTorque_=(v:Double) = javaSensorState.m_jointMotorTorque = v;

    val forceTorque:DMat = new DMat(1, 6, javaSensorState.m_jointForceTorque);

    def this() = this(new edu.berkeley.bid.bullet.JointSensorState);

    def unpack() = {
	(position, velocity, forceTorque, motorTorque)
    }
	
    override def toString ():String = {
    	"JointSensorState[position=" + position + ", " +
	"velocity=" + velocity + ", " +
	"forceTorque=" + forceTorque + ", " +
	"motorTorque=" + motorTorque + "]";
    }

}

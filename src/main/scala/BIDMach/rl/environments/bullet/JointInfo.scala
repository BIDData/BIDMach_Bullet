package BIDMach.rl.environments.bullet;
import BIDMat.{DMat}

@SerialVersionUID(45323457234254L)
class JointInfo(jji:edu.berkeley.bid.bullet.JointInfo) extends Serializable {
    val javaJointInfo = jji;

    def linkName = javaJointInfo.m_linkName;
    def jointName = javaJointInfo.m_jointName;
    def jointType = javaJointInfo.m_jointType;
    def qIndex = javaJointInfo.m_qIndex;
    def uIndex = javaJointInfo.m_uIndex;
    def jointIndex = javaJointInfo.m_jointIndex;
    def flags = javaJointInfo.m_flags;
    def jointDamping = javaJointInfo.m_jointDamping;
    def jointFriction = javaJointInfo.m_jointFriction;
    def jointLowerLimit = javaJointInfo.m_jointLowerLimit;
    def jointUpperLimit = javaJointInfo.m_jointUpperLimit;
    def jointMaxForce = javaJointInfo.m_jointMaxForce;
    def jointMaxVelocity = javaJointInfo.m_jointMaxVelocity;

    def linkName_=(v:String) = {javaJointInfo.m_linkName = v;};
    def jointName_=(v:String) = {javaJointInfo.m_jointName = v;};
    def jointType_=(v:Int) = {javaJointInfo.m_jointType = v;};
    def qIndex_=(v:Int) = {javaJointInfo.m_qIndex = v;};
    def uIndex_=(v:Int) = {javaJointInfo.m_uIndex = v;};
    def jointIndex_=(v:Int) = {javaJointInfo.m_jointIndex = v;};
    def flags_=(v:Int) = {javaJointInfo.m_flags = v;};
    def jointDamping_=(v:Double) = {javaJointInfo.m_jointDamping = v;};
    def jointFriction_=(v:Double) = {javaJointInfo.m_jointFriction = v;};
    def jointLowerLimit_=(v:Double) = {javaJointInfo.m_jointLowerLimit = v;};
    def jointUpperLimit_=(v:Double) = {javaJointInfo.m_jointUpperLimit = v;};
    def jointMaxForce_=(v:Double) = {javaJointInfo.m_jointMaxForce = v;};
    def jointMaxVelocity_=(v:Double) = {javaJointInfo.m_jointMaxVelocity = v;};

    val parentFrame = new DMat(1,7, javaJointInfo.m_parentFrame);
    val childFrame = new DMat(1,7, javaJointInfo.m_childFrame);
    val jointAxis = new DMat(1,7, javaJointInfo.m_jointAxis);

    def this() = this(new edu.berkeley.bid.bullet.JointInfo);


    override def toString () = {
	"[" +
	    "jointIndex=" + jointIndex + ", " +
	    "jointName=" + jointName + ", " +
	    "jointType=" + jointType + ", " +
	    "qIndex=" + qIndex + ", " +
	    "uIndex=" + uIndex + ", " +
	    "flags=" + flags + ", " +
	    "jointDamping=" + jointDamping + ", " +
	    "jointFriction=" + jointFriction + ", " +
	    "jointLowerLimit=" + jointLowerLimit + ", " +
	    "jointUpperLimit=" + jointUpperLimit + ", " +
	    "jointMaxForce=" + jointMaxForce + ", " +
	    "jointMaxVelocity=" + jointMaxVelocity + ", " +
	    "linkName=" + linkName + ", " +
	    "parentFrame=" + parentFrame + ", " +
	    "childFrame=" + childFrame + ", " +
	    "jointAxis=" + jointAxis + "]";
    }

}

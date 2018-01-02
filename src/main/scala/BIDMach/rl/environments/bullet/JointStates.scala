package BIDMach.rl.environments.bullet;
import BIDMat.{FMat,DMat};

@SerialVersionUID(32134214321342L)
class JointStates(jss:edu.berkeley.bid.bullet.JointStates2, numJoints:Int) extends Serializable {

    val javaJointStates = jss;

    def bodyUniqueId = javaJointStates.m_bodyUniqueId;
    def ndofQ = javaJointStates.m_numDegreeOfFreedomQ;
    def ndofU = javaJointStates.m_numDegreeOfFreedomU;

    def bodyUniqueId_=(v:Int) = {javaJointStates.m_bodyUniqueId = v;};
    def ndofQ_= (v:Int) = {javaJointStates.m_numDegreeOfFreedomQ = v;};
    def ndofU_=(v:Int) = {javaJointStates.m_numDegreeOfFreedomU = v;};

    val jointPositions = new DMat(1, ndofQ, javaJointStates.m_actualStateQ);
    val jointVelocities = new DMat(1, ndofU, javaJointStates.m_actualStateQdot);
    val jointReactionForces = new DMat(6, javaJointStates.m_jointReactionForces.length/6, javaJointStates.m_jointReactionForces);

    val rootLocalInertialFrame = new Transform3(javaJointStates.m_rootLocalInertialFrame);

    def unpack = {
	(jointPositions, jointVelocities, jointReactionForces)
    }

    override def toString () = {
	"JointStates[bodyUniqueId=" + bodyUniqueId + ", " +
	"ndofQ=" + ndofQ + ", " +
	"ndofU=" + ndofU + ", " +
	"rootLocalInertialFrame=" + rootLocalInertialFrame + ", " +
	"jointPositions=" + jointPositions + ", " +
	"jointVelocities=" + jointVelocities + ", " +
	"jointReactionForces=" + jointReactionForces + "]";
    }
}

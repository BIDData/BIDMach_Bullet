package BIDMach.rl.environments.bullet;
import BIDMat.DMat;

@SerialVersionUID(473252348962354L)
class DynamicsInfo(jdi:edu.berkeley.bid.bullet.DynamicsInfo) extends Serializable {
    val javaDynamicsInfo:edu.berkeley.bid.bullet.DynamicsInfo = jdi;
    val	localInertialPosition = new DMat(1, 3, javaDynamicsInfo.m_localInertialPosition);

    def mass = javaDynamicsInfo.m_mass;
    def lateralFrictionCoeff = javaDynamicsInfo.m_lateralFrictionCoeff;

    def mass_=(v:Double) = javaDynamicsInfo.m_mass = v;
    def lateralFrictionCoeff_=(v:Double) = javaDynamicsInfo.m_lateralFrictionCoeff = v;

    def this() = this(new edu.berkeley.bid.bullet.DynamicsInfo);

    override def toString () = {
       "DynamicsInfo[mass=" + mass +
       ",lateralFrictionCoeff=" + lateralFrictionCoeff +
       ",localInertialPosition=[" + localInertialPosition(0) + "," + localInertialPosition(1) + "," + localInertialPosition(2) + "]]";
    }

}

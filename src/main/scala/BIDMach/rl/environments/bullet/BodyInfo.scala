package BIDMach.rl.environments.bullet;

@SerialVersionUID(473252348962354L)
class BodyInfo(jbi:edu.berkeley.bid.bullet.BodyInfo) extends Serializable {
    val javaBodyInfo:edu.berkeley.bid.bullet.BodyInfo = jbi;

    def baseName:String = javaBodyInfo.m_baseName;
    def bodyName:String = javaBodyInfo.m_bodyName;

    def baseName_=(s:String) = javaBodyInfo.m_baseName = s;
    def bodyName_=(s:String) = javaBodyInfo.m_bodyName = s;

    def this() = this(new edu.berkeley.bid.bullet.BodyInfo());
	
    override def toString () = {
	"BodyInfo[baseName=" + baseName + ",bodyName=" + bodyName + "]";
    }

}

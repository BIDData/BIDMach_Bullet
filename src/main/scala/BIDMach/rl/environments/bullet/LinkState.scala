package BIDMach.rl.environments.bullet;
import BIDMat.DMat;

@SerialVersionUID(3123427891283132L)
class LinkState(jls:edu.berkeley.bid.bullet.LinkState) extends Serializable {
    val javaLinkState = jls;

    val worldPosition = new DMat(1, 3, javaLinkState.m_worldPosition);
    val worldOrientation = new DMat(1, 4, javaLinkState.m_worldOrientation);
    val localInertialPosition = new DMat(1, 3, javaLinkState.m_localInertialPosition);
    val localInertialOrientation = new DMat(1, 4, javaLinkState.m_localInertialOrientation);
    val worldLinkFramePosition = new DMat(1, 3, javaLinkState.m_worldLinkFramePosition);
    val worldLinkFrameOrientation = new DMat(1, 4, javaLinkState.m_worldLinkFrameOrientation);
    val worldLinearVelocity = new DMat(1, 3, javaLinkState.m_worldLinearVelocity);
    val worldAngularVelocity = new DMat(1, 3, javaLinkState.m_worldAngularVelocity);
    val worldAABBMin = new DMat(1, 3, javaLinkState.m_worldAABBMin);
    val worldAABBMax = new DMat(1, 3, javaLinkState.m_worldAABBMax);

    def this() = this(new edu.berkeley.bid.bullet.LinkState());

    def unpack = {
	(worldPosition, worldOrientation,
	 localInertialPosition, localInertialOrientation,
	 worldLinkFramePosition, worldLinkFrameOrientation,
	 worldLinearVelocity, worldAngularVelocity,
	 worldAABBMin, worldAABBMax);
    }
	
    override def toString () = {
	"LinkState[worldPosition=(" + worldPosition(0) + "," + worldPosition(1) + "," + worldPosition(2) + ")," +
	"worldOrientation=(" + worldOrientation(0) + "," + worldOrientation(1) + "," + worldOrientation(2) + "," + worldOrientation(3) + ")," +
	"localInertialPosition=(" + localInertialPosition(0) + "," + localInertialPosition(1) + "," + localInertialPosition(2) + ")," +
	"localInertialOrientation=(" + localInertialOrientation(0) + "," + localInertialOrientation(1) + "," + localInertialOrientation(2) + "," + localInertialOrientation(3) + ")," +
	"worldLinkFramePosition=(" + worldLinkFramePosition(0) + "," + worldLinkFramePosition(1) + "," + worldLinkFramePosition(2) + ")," +
	"worldLinkFrameOrientation=(" + worldLinkFrameOrientation(0) + "," + worldLinkFrameOrientation(1) + "," + worldLinkFrameOrientation(2) + "," + worldLinkFrameOrientation(3) + ")," +
	"worldLinearVelocity=(" + worldLinearVelocity(0) + "," + worldLinearVelocity(1) + "," + worldLinearVelocity(2) + ")," +
	"worldAngularVelocity=(" + worldAngularVelocity(0) + "," + worldAngularVelocity(1) + "," + worldAngularVelocity(2) + ")," +
	"worldAABBMin=(" + worldAABBMin(0) + "," + worldAABBMin(1) + "," + worldAABBMin(2) + ")," +
	"worldAABBMax=(" + worldAABBMax(0) + "," + worldAABBMax(1) + "," + worldAABBMax(2) + ")]";
    }

}

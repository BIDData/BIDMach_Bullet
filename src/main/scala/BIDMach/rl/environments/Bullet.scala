package BIDMach.rl.environments;
import edu.berkeley.bid.bullet._;

class Bullet extends edu.berkeley.bid.Bullet {
	
    def getBasePositionAndOrientation(bodyUniqueId:Int):(Vector3, Quaternion) = {
	val basePosition = new Vector3();
	val baseOrientation = new Quaternion();
	getBasePositionAndOrientation(bodyUniqueId, basePosition, baseOrientation);
	(basePosition, baseOrientation);
    };

    def getBaseVelocity(bodyUniqueId:Int):(Vector3, Vector3) = {
	val baseVelocity = new Vector3();
	val baseAngularVelocity = new Vector3
	getBaseVelocity(bodyUniqueId, baseVelocity, baseAngularVelocity);
	(baseVelocity, baseAngularVelocity);
    }
}

object Bullet {

    def getQuaternionFromEuler(euler:Vector3) = {
	val q = new Quaternion();
	edu.berkeley.bid.Bullet.getQuaternionFromEuler(euler, q);
	q;
    };

    def getEulerFromQuaternion(q:Quaternion) = {
	val v = new Vector3();
	edu.berkeley.bid.Bullet.getEulerFromQuaternion(q, v);
	v;
    }

    final val eCONNECT_GUI = 1;
    final val eCONNECT_DIRECT = 2;
    final val eCONNECT_SHARED_MEMORY = 3;
    final val eCONNECT_UDP = 4;
    final val eCONNECT_TCP = 5;
    final val eCONNECT_EXISTING_EXAMPLE_BROWSER=6;
    final val eCONNECT_GUI_SERVER=7;

    final val COV_ENABLE_GUI=1;
    final val COV_ENABLE_SHADOWS=2;
    final val COV_ENABLE_WIREFRAME=3;
    final val COV_ENABLE_VR_TELEPORTING=4;
    final val COV_ENABLE_VR_PICKING=5;
    final val COV_ENABLE_VR_RENDER_CONTROLLERS=6;
    final val COV_ENABLE_RENDERING=7;
    final val COV_ENABLE_SYNC_RENDERING_INTERNAL=8;
    final val COV_ENABLE_KEYBOARD_SHORTCUTS=9;
    final val COV_ENABLE_MOUSE_PICKING=10;
    final val COV_ENABLE_Y_AXIS_UP=11;
    final val COV_ENABLE_TINY_RENDERER=12;
    final val COV_ENABLE_RGB_BUFFER_PREVIEW=13;
    final val COV_ENABLE_DEPTH_BUFFER_PREVIEW=14;
    final val COV_ENABLE_SEGMENTATION_MARK_PREVIEW=15;

    final val STATE_LOGGING_MINITAUR = 0;
    final val STATE_LOGGING_GENERIC_ROBOT = 1;
    final val STATE_LOGGING_VR_CONTROLLERS = 2;
    final val STATE_LOGGING_VIDEO_MP4 = 3;
    final val STATE_LOGGING_COMMANDS = 4;
    final val STATE_LOGGING_CONTACT_POINTS = 5;
    final val STATE_LOGGING_PROFILE_TIMINGS = 6;

    final val CONTROL_MODE_VELOCITY=0;
    final val CONTROL_MODE_TORQUE=1;
    final val CONTROL_MODE_POSITION_VELOCITY_PD=2;

    final val IK_DLS=0;
    final val IK_SDLS=1;
    final val IK_HAS_TARGET_POSITION=16;
    final val IK_HAS_TARGET_ORIENTATION=32;
    final val IK_HAS_NULL_SPACE_VELOCITY=64;
    final val IK_HAS_JOINT_DAMPING=128;

    final val EF_LINK_FRAME=1;
    final val EF_WORLD_FRAME=2;

    final val DEB_DEBUG_TEXT_ALWAYS_FACE_CAMERA=1;
    final val DEB_DEBUG_TEXT_USE_TRUE_TYPE_FONTS=2;
    final val DEB_DEBUG_TEXT_HAS_TRACKING_OBJECT=4;

    final val URDF_USE_INERTIA_FROM_FILE=2;
    final val URDF_USE_SELF_COLLISION=8;
    final val URDF_USE_SELF_COLLISION_EXCLUDE_PARENT=16;
    final val URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS=32;
    final val URDF_RESERVED=64;

    final val GEOM_SPHERE=2;
    final val GEOM_BOX=3;
    final val GEOM_CYLINDER=4;
    final val GEOM_MESH=5;
    final val GEOM_PLANE=6;
    final val GEOM_CAPSULE=7;
    final val GEOM_UNKNOWN=8; 

    final val GEOM_FORCE_CONCAVE_TRIMESH=1;
    
    final val GEOM_VISUAL_HAS_RGBA_COLOR=1;
    final val GEOM_VISUAL_HAS_SPECULAR_COLOR=2;

    final val STATE_LOG_JOINT_MOTOR_TORQUES=1;
    final val STATE_LOG_JOINT_USER_TORQUES=2;
    final val STATE_LOG_JOINT_TORQUES = STATE_LOG_JOINT_MOTOR_TORQUES+STATE_LOG_JOINT_USER_TORQUES;

}

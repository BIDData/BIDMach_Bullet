package edu.berkeley.bid;
import edu.berkeley.bid.bullet.*;
import java.io.*;

public class Bullet implements Serializable {
	
    static final long serialVersionUID = 5233456234785623856L;

    static {
	LibUtils.loadLibrary("bidmachbullet", false);
    }

    private long handle = 0;

    public Bullet() {
	newRobotSimulatorClientAPI();
    }

    protected void finalize() {
        if (handle != 0) {
            deleteRobotSimulatorClientAPI();
            handle = 0;
        }
    }

    public static native void testMatrix3x3(Matrix3x3 min, Matrix3x3 mout);

    public static native void testTransform3(Transform3 min, Transform3 mout);

    public static native void testJointInfo(JointInfo min, JointInfo mout);

    public static native void testJointSensorState(JointSensorState min, JointSensorState mout);

    public static native void testJointStates2(JointStates2 min, JointStates2 mout, int numJoints);

    public static native void testJointMotorArgs(JointMotorArgs min, JointMotorArgs mout);

    private native int newRobotSimulatorClientAPI();

    private native int deleteRobotSimulatorClientAPI();

    public native void renderScence();

    public native void debugDraw();

    public native boolean connect(int method, String hostname, int port);

    public boolean connect(int method, String hostname) {
	return connect(method, hostname, -1);
    }

    public boolean connect(int method) {
	return connect(method, "localhost", -1);
    }

    public native void disconnect();

    public native boolean isConnected();

    public native void setTimeOut(double t);

    public native void syncBodies();

    public native void resetSimulation();

    public native void getQuaternionFromEuler(Vector3 euler, Quaternion q);

    public Quaternion getQuaternionFromEuler(Vector3 euler) {
	Quaternion q = new Quaternion();
	getQuaternionFromEuler(euler, q);
	return q;
    }

    public native void getEulerFromQuaternion(Quaternion q, Vector3 euler);

    public Vector3 getEulerFromQuaternion(Quaternion q) {
	Vector3 v = new Vector3();
	getEulerFromQuaternion(q, v);
	return v;
    }


    public native int loadURDF(String fname, Vector3 startPos, Quaternion startOrient, boolean forceOverrideFixedBase, boolean useMultiBody, int flags);

    public int loadURDF(String fname, Vector3 startPos, Quaternion startOrient) {
	return loadURDF(fname, startPos, startOrient, false, true, 0);
    }

    public int loadURDF(String fname) {
	return loadURDF(fname, new Vector3(), new Quaternion(), false, true, 0);
    }

    public native int[] loadSDF(String fname, boolean forceOverrideFixedBase, boolean useMultiBody);

    public int[] loadSDF(String fname) {
	return loadSDF(fname, false, true);
    }

    public native int[] loadMJCF(String fname);

    public native int[] loadBullet(String fname);

    public native void configureDebugVisualizer(int flags, int enable);

    public native boolean getBodyInfo(int bodyUniqueId, BodyInfo bodyInfo);

    public native boolean getBasePositionAndOrientation(int bodyUniqueId, Vector3 basePosition, Quaternion baseOrientation);

    public native boolean resetBasePositionAndOrientation(int bodyUniqueId, Vector3 basePosition, Quaternion baseOrientation);

    public native boolean getBaseVelocity(int bodyUniqueId, Vector3 baseVelocity, Vector3 baseAngularV);

    public native boolean resetBaseVelocity(int bodyUniqueId, Vector3 baseVelocity, Vector3 baseAngularV);

    public native int getNumJoints(int bodyUniqueId);

    public native boolean getJointInfo(int bodyUniqueId, int jointIndex, JointInfo jointInfo);

    public native int createConstraint(int parentBodyIndex, int parentJointIndex, int childBodyIndex, int childJointIndex, JointInfo jointInfo);

    public native int changeConstraint(int constraintId, JointInfo jointInfo);

    public native void removeConstraint(int constraintId);

    public native boolean getJointState(int bodyUniqueId, int jointIndex, JointSensorState state);

    public native boolean getJointStates(int bodyUniqueId, JointStates2 state);

    public native boolean resetJointState(int bodyUniqueId, int jointIndex, double targetValue);

    public native void setJointMotorControl(int bodyUniqueId, int jointIndex, JointMotorArgs args);

    public native void stepSimulation();

    public native boolean canSubmitCommand();

    public native void setRealTimeSimulation(boolean enable);

    public native void setInternalSimFlags(int flags);

    public native void setGravity(Vector3 gravity);

    public native void setTimeStep(double t);

    public native void setNumSimulationSubSteps(int numSubSteps);

    public native void setNumSolverIterations(int numSolverIterations);

    public native void setContactBreakingThreshold(double threshold);

    public static final int eCONNECT_GUI = 1;
    public static final int eCONNECT_DIRECT = 2;
    public static final int eCONNECT_SHARED_MEMORY = 3;
    public static final int eCONNECT_UDP = 4;
    public static final int eCONNECT_TCP = 5;
    public static final int eCONNECT_EXISTING_EXAMPLE_BROWSER=6;
    public static final int eCONNECT_GUI_SERVER=7;

    public static final int COV_ENABLE_GUI=1;
    public static final int COV_ENABLE_SHADOWS=2;
    public static final int COV_ENABLE_WIREFRAME=3;
    public static final int COV_ENABLE_VR_TELEPORTING=4;
    public static final int COV_ENABLE_VR_PICKING=5;
    public static final int COV_ENABLE_VR_RENDER_CONTROLLERS=6;
    public static final int COV_ENABLE_RENDERING=7;
    public static final int COV_ENABLE_SYNC_RENDERING_INTERNAL=8;
    public static final int COV_ENABLE_KEYBOARD_SHORTCUTS=9;
    public static final int COV_ENABLE_MOUSE_PICKING=10;
    public static final int COV_ENABLE_Y_AXIS_UP=11;
    public static final int COV_ENABLE_TINY_RENDERER=12;
    public static final int COV_ENABLE_RGB_BUFFER_PREVIEW=13;
    public static final int COV_ENABLE_DEPTH_BUFFER_PREVIEW=14;
    public static final int COV_ENABLE_SEGMENTATION_MARK_PREVIEW=15;

}

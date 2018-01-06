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

    private native int newRobotSimulatorClientAPI();

    private native int deleteRobotSimulatorClientAPI();

    public native boolean connect(int method, String hostname, int port);

    public boolean connect(int method, String hostname) {
	return connect(method, hostname, -1);
    }

    public boolean connect(int method) {
	return connect(method, "localhost", -1);
    }

    public native void disconnect();

    public native boolean isConnected();

    public native void setGravity(Vector3 gravity);

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

    public native void stepSimulation();

    public native void setRealTimeSimulation(boolean enable);

    public native boolean getBasePositionAndOrientation(int bodyUniqueId, Vector3 basePosition, Quaternion baseOrientation);

    public native boolean resetBasePositionAndOrientation(int bodyUniqueId, Vector3 basePosition, Quaternion baseOrientation);

    public static native void getQuaternionFromEuler(Vector3 euler, Quaternion q);

    public static Quaternion getQuaternionFromEuler(Vector3 euler) {
	Quaternion q = new Quaternion();
	getQuaternionFromEuler(euler, q);
	return q;
    }

    public static native void getEulerFromQuaternion(Quaternion q, Vector3 euler);

    public static Vector3 getEulerFromQuaternion(Quaternion q) {
	Vector3 v = new Vector3();
	getEulerFromQuaternion(q, v);
	return v;
    }

    public native int getNumJoints(int bodyUniqueId);

    public native boolean getJointInfo(int bodyUniqueId, int jointIndex, JointInfo jointInfo);

    public JointInfo getJointInfo(int bodyUniqueId, int jointIndex) {
	JointInfo jointInfo = new JointInfo();
	getJointInfo(bodyUniqueId, jointIndex, jointInfo);
	return jointInfo;
    }

    public native boolean getJointState(int bodyUniqueId, int jointIndex, JointSensorState state);

    public JointSensorState getJointState(int bodyUniqueId, int jointIndex) {
	JointSensorState jointState = new JointSensorState();
	getJointState(bodyUniqueId, jointIndex, jointState);
	return jointState;
    }

    public native boolean resetJointState(int bodyUniqueId, int jointIndex, double targetValue);

    public native boolean getJointStates(int bodyUniqueId, JointStates2 state);

    public JointStates2 getJointStates(int bodyUniqueId) {
	JointStates2 jointStates = new JointStates2();
	getJointStates(bodyUniqueId, jointStates);
	return jointStates;
    }

    public void getJointStates(int bodyUniqueId, int [] jointIndices,
			       double [] jointPositions, double [] jointVelocities,
			       double [] jointForceTorque, double [] jointMotorTorque) {
	int numjoints = jointIndices.length;
	int forceTorqueSize = 6; 
	for (int i = 0; i < numjoints; i++) {
	    JointSensorState jointState = getJointState(bodyUniqueId, i);
	    jointPositions[i] = jointState.m_jointPosition;
	    jointVelocities[i] = jointState.m_jointVelocity;
	    jointMotorTorque[i] = jointState.m_jointMotorTorque;
	    jointForceTorque[forceTorqueSize * i] = jointState.m_jointForceTorque[0];
	    jointForceTorque[forceTorqueSize * i + 1] = jointState.m_jointForceTorque[1];
	    jointForceTorque[forceTorqueSize * i + 2] = jointState.m_jointForceTorque[2];
	    jointForceTorque[forceTorqueSize * i + 2] = jointState.m_jointForceTorque[3];
	    jointForceTorque[forceTorqueSize * i + 4] = jointState.m_jointForceTorque[4];
	    jointForceTorque[forceTorqueSize * i + 5] = jointState.m_jointForceTorque[5];
	}
    }

    public native boolean enableJointForceTorqueSensor(int bodyUniqueId, int jointIndex, boolean enable);

    public native void setJointMotorControl(int bodyUniqueId, int jointIndex,
					    int controlMode,  double targetPosition, double targetVelocity,
					    double force, double kp, double kd);

    public native boolean setJointMotorControlArray(int bodyUniqueId, int [] jointIndices, int controlMode,
						    double [] targetPositions, double [] targetVelocities, double [] maxTorques, double [] kps, double [] kds);
    
    public native boolean getLinkState(int bodyUniqueId, int linkIndex, int computeLinkVelocity, int computeForwardKinematics, LinkState linkState);

    public LinkState getLinkState(int bodyUniqueId, int linkIndex, int computeLinkVelocity, int computeForwardKinematics) {
	LinkState linkState = new LinkState();
	getLinkState(bodyUniqueId, linkIndex, computeLinkVelocity, computeForwardKinematics, linkState);
	return linkState;
    }

    public native boolean getBaseVelocity(int bodyUniqueId, Vector3 baseVelocity, Vector3 baseAngularV);

    public native boolean resetBaseVelocity(int bodyUniqueId, Vector3 baseVelocity, Vector3 baseAngularV);

    public native int getNumBodies();

    public native int getBodyUniqueId(int bodyId);

    public native boolean removeBody(int bodyUniqeId);

    public native boolean getBodyInfo(int bodyUniqueId, BodyInfo bodyInfo);

    public BodyInfo getBodyInfo(int bodyUniqueId) {
	BodyInfo bodyInfo = new BodyInfo();
	getBodyInfo(bodyUniqueId, bodyInfo);
	return bodyInfo;
    }

    public native boolean applyExternalForce(int objectUniqueId, int linkIndex, double [] force, double [] position, int flags);

    public native boolean applyExternalTorque(int objectUniqueId, int linkIndex, double [] torque, int flags);

    public native int createConstraint(int parentBodyIndex, int parentJointIndex, int childBodyIndex, int childJointIndex, JointInfo jointInfo);

    public int createConstraint(int parentBodyIndex, int parentJointIndex, int childBodyIndex, int childJointIndex,
				int jointType, double [] jointAxis, double [] parentFramePosition, double [] childFramePosition,
				double [] parentFrameOrientation, double [] childFrameOrientation) {

	JointInfo jointInfo = new JointInfo("","");
	jointInfo.m_jointType = jointType;

	jointInfo.m_parentFrame[0] = parentFramePosition[0];
	jointInfo.m_parentFrame[1] = parentFramePosition[1];
	jointInfo.m_parentFrame[2] = parentFramePosition[2];
	
	jointInfo.m_parentFrame[3] = parentFrameOrientation[0];
	jointInfo.m_parentFrame[4] = parentFrameOrientation[1];
	jointInfo.m_parentFrame[5] = parentFrameOrientation[2];
	jointInfo.m_parentFrame[6] = parentFrameOrientation[3];

	jointInfo.m_childFrame[0] = childFramePosition[0];
	jointInfo.m_childFrame[1] = childFramePosition[1];
	jointInfo.m_childFrame[2] = childFramePosition[2];
	
	jointInfo.m_childFrame[3] = childFrameOrientation[0];
	jointInfo.m_childFrame[4] = childFrameOrientation[1];
	jointInfo.m_childFrame[5] = childFrameOrientation[2];
	jointInfo.m_childFrame[6] = childFrameOrientation[3];

	jointInfo.m_jointAxis[0] = jointAxis[0];
	jointInfo.m_jointAxis[1] = jointAxis[1];
	jointInfo.m_jointAxis[2] = jointAxis[2];

	return createConstraint(parentBodyIndex, parentJointIndex, childBodyIndex, childJointIndex, jointInfo);
    }


    public native int changeConstraint(int constraintId, JointInfo jointInfo);

    public native void removeConstraint(int constraintId);

    public native void setTimeStep(double t);

    public native void setInternalSimFlags(int flags);

    public native void setNumSimulationSubSteps(int numSubSteps);

    public native void setNumSolverIterations(int numSolverIterations);

    public native void setContactBreakingThreshold(double threshold);

    public native void resetSimulation();

    public native void startStateLogging(int loggingType, String fileName, int [] objectUniqueIds, int maxLogDof);

    public native void stopStateLogging(int stateLoggerUniqueId);

    public static native void computeViewMatrixFromPositions(float [] jcameraPosition, float [] jcameraTargetPosition,
							     float [] jcameraUp, float [] jviewMatrix);

    public static native void computeViewMatrixFromYawPitchRoll(float [] cameraTargetPosition, float distance, float yaw, float pitch, float roll,
								int upAxisIndex, float [] viewMatrix);
    
    public static native void computeProjectionMatrix(float left, float right, float bottom, float top,
						      float nearVal, float farVal, float [] jprojectionMatrix);

    public static native void computeProjectionMatrixFOV(float fov, float aspect, float nearVal, float farVal, float [] jprojectionMatrix);

    public native boolean getCameraImage(int width, int height,
					 float [] viewMatrix, float [] projectionMatrix,
					 float [] lightProjection, float [] lightColor,
					 float lightDistance, int hasShadow,
					 float lightAmbientCoeff, float lightDiffuseCoeff, float lightSpecularCoeff,
					 int renderer, CameraImageData cameraImage);

    public native boolean getCameraImageInts(int width, int height,
					     float [] viewMatrix, float [] projectionMatrix,
					     float [] lightProjection, float [] lightColor,
					     float lightDistance, int hasShadow,
					     float lightAmbientCoeff, float lightDiffuseCoeff, float lightSpecularCoeff,
					     int renderer, int [] rgbColorData, float [] depthValues, int [] segmentationValues);

    public native boolean getCameraImageBytes(int width, int height,
					      float [] viewMatrix, float [] projectionMatrix,
					      float [] lightProjection, float [] lightColor,
					      float lightDistance, int hasShadow,
					      float lightAmbientCoeff, float lightDiffuseCoeff, float lightSpecularCoeff,
					      int renderer, byte [] rgbColorData, float [] depthValues, int [] segmentationValues);

    public native boolean calculateInverseDynamics(int bodyUniqueId, double [] jointPositions, double [] jointVelocities,
						   double [] jointAccelerations, double [] jointForcesOutput) ;

    public double [] calculateInverseDynamics(int bodyUniqueId, double [] jointPositions, double [] jointVelocities, double [] jointAccelerations) {
	int numJoints = getNumJoints(bodyUniqueId);
	double [] jointForcesOutput = new double[numJoints];
	calculateInverseDynamics(bodyUniqueId, jointPositions, jointVelocities, jointAccelerations, jointForcesOutput);
	return jointForcesOutput;
    }

    public native boolean getBodyJacobian(int bodyUniqueId, int linkIndex, double [] localPosition, double [] jointPositions, double [] jointVelocities,
					  double [] jointAccelerations, double [] linearJacobian, double [] angularJacobian);

    public boolean calculateJacobian(int bodyUniqueId, int linkIndex, double [] localPosition, double [] jointPositions, double [] jointVelocities,
				     double [] jointAccelerations, double [] linearJacobian, double [] angularJacobian) {
	return getBodyJacobian(bodyUniqueId, linkIndex, localPosition, jointPositions, jointVelocities,
					  jointAccelerations, linearJacobian, angularJacobian);
    }

    public native boolean calculateInverseKinematics(int bodyUniqueId, int endEffectorLinkIndex,
						     double [] endEffectorTargetPosition, double [] endEffectorTargetOrientation, 
						     double [] lowerLimits, double [] upperLimits, double [] jointRanges, double [] restPoses,
						     double [] jointDamping, double [] jointAnglesOutput);

    public double [] calculateInverseKinematics(int bodyUniqueId, int endEffectorLinkIndex,
						double [] endEffectorTargetPosition, double [] endEffectorTargetOrientation, 
						double [] lowerLimits, double [] upperLimits, double [] jointRanges, double [] restPoses,
						double [] jointDamping) {
	int numJoints = getNumJoints(bodyUniqueId);
	double [] jointAnglesOutput = new double[numJoints];
	calculateInverseKinematics(bodyUniqueId, endEffectorLinkIndex, endEffectorTargetPosition, endEffectorTargetOrientation, 
				   lowerLimits, upperLimits, jointRanges, restPoses,
				   jointDamping, jointAnglesOutput);
	return jointAnglesOutput;
    }

    public native boolean getDynamicsInfo(int bodyUniqueId, int jointIndex, DynamicsInfo dynamicsInfo);

    public native boolean changeDynamics(int bodyUniqueId, int linkIndex, double mass, double lateralFriction, double spinningFriction,
					 double rollingFriction, double restitution, double linearDamping, double angularDamping,
					 double contactStiffness, double contactDamping, int frictionAnchor);

    public native void renderScene();

    public native void debugDraw();

    public native void setTimeOut(double t);

    public native void syncBodies();

    public native boolean canSubmitCommand();

    public native void configureDebugVisualizer(int flags, int enable);

    public native void resetDebugVisualizerCamera(double cameraDistance, double cameraPitch, double cameraYaw, Vector3 targetPos);

    public native void getKeyboardEventsData(KeyboardEventsData keyboardEventsData);

    public KeyboardEventsData getKeyboardEventsData() {
	KeyboardEventsData keyboardEventsData = new KeyboardEventsData();
	getKeyboardEventsData(keyboardEventsData);
	return keyboardEventsData;
    }

    public native void submitProfileTiming(String profileName, int durationInMicroseconds);

    public native int addUserDebugLine(double [] fromXYZ, double [] toXYZ, double [] colorRGB,
				       double lineWidth, double lifeTime, int parentObjectUniqueId, int parentLinkIndex);

    public native int addUserDebugText3D(String text, double [] position, double [] orientation, double [] colorRGB,
					 double size, double lifeTime, int parentObjectUniqueId, int parentLinkIndex);

    public native int addUserDebugParameter(String paramName, double rangeMin, double rangeMax, double startValue);

    public native double readUserDebugParameter(int itemUniqueId);

    public native boolean removeUserDebugItem(int itemUniqueId);

    public native boolean setPhysicsEngineParameter(double fixedTimeStep, int numSolverIterations, int useSplitImpulse,
						    double splitImpulsePenetrationThreshold, int numSubSteps,
						    int collisionFilterMode, double contactBreakingThreshold,
						    int maxNumCmdPer1ms, int enableFileCaching, double restitutionVelocityThreshold,
						    double erp, double contactERP, double frictionERP);


    public static native void testMatrix3x3(Matrix3x3 min, Matrix3x3 mout);

    public static native void testTransform3(Transform3 min, Transform3 mout);

    public static native void testJointInfo(JointInfo min, JointInfo mout);

    public static native void testJointSensorState(JointSensorState min, JointSensorState mout);

    public static native void testJointStates2(JointStates2 min, JointStates2 mout, int numJoints);

    public static native void testLinkState(LinkState min, LinkState mout);

    public static native void testKeyboardEventsData(KeyboardEventsData min, KeyboardEventsData mout);

    public static native void testCameraImageData(CameraImageData min, CameraImageData mout);

    public static native void testDynamicsInfo(DynamicsInfo min, DynamicsInfo mout);


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

    public static final int STATE_LOGGING_MINITAUR = 0;
    public static final int STATE_LOGGING_GENERIC_ROBOT = 1;
    public static final int STATE_LOGGING_VR_CONTROLLERS = 2;
    public static final int STATE_LOGGING_VIDEO_MP4 = 3;
    public static final int STATE_LOGGING_COMMANDS = 4;
    public static final int STATE_LOGGING_CONTACT_POINTS = 5;
    public static final int STATE_LOGGING_PROFILE_TIMINGS = 6;

    public static final int CONTROL_MODE_VELOCITY=0;
    public static final int CONTROL_MODE_TORQUE=1;
    public static final int CONTROL_MODE_POSITION_VELOCITY_PD=2;

    public static final int IK_DLS=0;
    public static final int IK_SDLS=1;
    public static final int IK_HAS_TARGET_POSITION=16;
    public static final int IK_HAS_TARGET_ORIENTATION=32;
    public static final int IK_HAS_NULL_SPACE_VELOCITY=64;
    public static final int IK_HAS_JOINT_DAMPING=128;

    public static final int EF_LINK_FRAME=1;
    public static final int EF_WORLD_FRAME=2;

    public static final int DEB_DEBUG_TEXT_ALWAYS_FACE_CAMERA=1;
    public static final int DEB_DEBUG_TEXT_USE_TRUE_TYPE_FONTS=2;
    public static final int DEB_DEBUG_TEXT_HAS_TRACKING_OBJECT=4;

    public static final int URDF_USE_INERTIA_FROM_FILE=2;
    public static final int URDF_USE_SELF_COLLISION=8;
    public static final int URDF_USE_SELF_COLLISION_EXCLUDE_PARENT=16;
    public static final int URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS=32;
    public static final int URDF_RESERVED=64;

    public static final int GEOM_SPHERE=2;
    public static final int GEOM_BOX=3;
    public static final int GEOM_CYLINDER=4;
    public static final int GEOM_MESH=5;
    public static final int GEOM_PLANE=6;
    public static final int GEOM_CAPSULE=7;
    public static final int GEOM_UNKNOWN=8; 

    public static final int GEOM_FORCE_CONCAVE_TRIMESH=1;
    
    public static final int GEOM_VISUAL_HAS_RGBA_COLOR=1;
    public static final int GEOM_VISUAL_HAS_SPECULAR_COLOR=2;

    public static final int STATE_LOG_JOINT_MOTOR_TORQUES=1;
    public static final int STATE_LOG_JOINT_USER_TORQUES=2;
    public static final int STATE_LOG_JOINT_TORQUES = STATE_LOG_JOINT_MOTOR_TORQUES+STATE_LOG_JOINT_USER_TORQUES;
}

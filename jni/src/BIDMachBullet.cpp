#include <jni.h>
#include <../examples/RobotSimulator/b3RobotSimulatorClientAPI.h>
#include <string.h>

union VoidLong {
  jlong l;
  void* p;
};

static jlong void2long(void* ptr) {
  union VoidLong v;
  v.l = (jlong) 0; 
  v.p = ptr;
  return v.l;
}

static void* long2void(jlong l) {
  union VoidLong v;
  v.l = l;
  return v.p;
}

static b3RobotSimulatorClientAPI *getRobotSimulatorClientAPI(JNIEnv *env, jobject jRoboSimAPI)
{
  static jclass clazz = NULL;
  static jfieldID handle_id = NULL;
  if (clazz == NULL) {
    clazz = (jclass) env->NewGlobalRef(env->FindClass("edu/berkeley/bid/Bullet"));
  }
  if (handle_id == NULL) {
    handle_id = env->GetFieldID(clazz, "handle", "J");
  }
  jlong handle = env->GetLongField(jRoboSimAPI, handle_id);
  b3RobotSimulatorClientAPI * pch = (b3RobotSimulatorClientAPI *)long2void(handle);
  return pch;
}

static void setRobotSimulatorClientAPI(JNIEnv *env, jobject jRoboSimAPI, b3RobotSimulatorClientAPI *pch)
{
  jclass clazz = env->GetObjectClass(jRoboSimAPI);
  jfieldID handle_id = env->GetFieldID(clazz, "handle", "J");
  jlong handle = void2long(pch);
  env->SetLongField(jRoboSimAPI, handle_id, handle);
}

#define CHECKFIELD(FIELDID,EXPR,MSG,RETV) \
  jfieldID FIELDID = (EXPR); \
  if (FIELDID == NULL) { \
    fprintf(stderr,MSG); \
    return RETV; \
  }

#define CHECKVALUE(VALUEID,MSG,RETV) \
  if (VALUEID == NULL) { \
    fprintf(stderr,MSG); \
    return RETV; \
  }


static void getQuaternionFields(JNIEnv *env, jclass &qclass, jfieldID &qxfield, jfieldID &qyfield, jfieldID &qzfield, jfieldID &qwfield)
{
  static jclass jqclass = NULL;
  static jfieldID jxfield = NULL;
  static jfieldID jyfield = NULL;
  static jfieldID jzfield = NULL;
  static jfieldID jwfield = NULL;
  
  if (jqclass == NULL) {
    jqclass = (jclass) env->NewGlobalRef(env->FindClass("edu/berkeley/bid/bullet/Quaternion"));
  }
  if (jqclass != NULL && jxfield == NULL) {
    jxfield = env->GetFieldID(jqclass, "x", "F");
    jyfield = env->GetFieldID(jqclass, "y", "F");
    jzfield = env->GetFieldID(jqclass, "z", "F");
    jwfield = env->GetFieldID(jqclass, "w", "F");
  }
  if (jxfield == NULL) {
    fprintf(stderr, "Couldnt initialize native Quaternion fields\n");
  }
  qclass = jqclass;
  qxfield = jxfield;
  qyfield = jyfield;
  qzfield = jzfield;
  qwfield = jwfield;
}

static b3Quaternion javaQuaternionToNative(JNIEnv *env, jobject jq) {
  jclass jqclass = NULL;
  jfieldID jxfield = NULL, jyfield = NULL, jzfield = NULL, jwfield = NULL;
  float x, y, z, w;
  getQuaternionFields(env, jqclass, jxfield, jyfield, jzfield, jwfield);
  x = env->GetFloatField(jq, jxfield);
  y = env->GetFloatField(jq, jyfield);
  z = env->GetFloatField(jq, jzfield);
  w = env->GetFloatField(jq, jwfield);
  return b3Quaternion(x, y, z, w);
}

static void nativeQuaternionToJava(JNIEnv *env, jobject jq, b3Quaternion &q) {
  jclass jqclass = NULL;
  jfieldID jxfield = NULL, jyfield = NULL, jzfield = NULL, jwfield = NULL;
  getQuaternionFields(env, jqclass, jxfield, jyfield, jzfield, jwfield);
  env->SetFloatField(jq, jxfield, q.x);
  env->SetFloatField(jq, jyfield, q.y);
  env->SetFloatField(jq, jzfield, q.z);
  env->SetFloatField(jq, jwfield, q.w);
}

static void getVector3Fields(JNIEnv *env, jclass &vclass, jfieldID &vxfield, jfieldID &vyfield, jfieldID &vzfield)
{
  static jclass jvclass = NULL;
  static jfieldID jxfield = NULL;
  static jfieldID jyfield = NULL;
  static jfieldID jzfield = NULL;
  
  if (jvclass == NULL) {
    jvclass = (jclass) env->NewGlobalRef(env->FindClass("edu/berkeley/bid/bullet/Vector3"));
  }
  if (jvclass != NULL && jxfield == NULL) {
    jxfield = env->GetFieldID(jvclass, "x", "F");
    jyfield = env->GetFieldID(jvclass, "y", "F");
    jzfield = env->GetFieldID(jvclass, "z", "F");
  }
  if (jxfield == NULL) {
    fprintf(stderr, "Couldnt initialize native Vector3 fields\n");
  }
  vclass = jvclass;
  vxfield = jxfield;
  vyfield = jyfield;
  vzfield = jzfield;
}

static b3Vector3 javaVector3ToNative(JNIEnv *env, jobject jv) {
  jclass jvclass = NULL;
  jfieldID jxfield = NULL, jyfield = NULL, jzfield = NULL;
  float x, y, z;
  getVector3Fields(env, jvclass, jxfield, jyfield, jzfield);
  x = env->GetFloatField(jv, jxfield);
  y = env->GetFloatField(jv, jyfield);
  z = env->GetFloatField(jv, jzfield);
  return b3MakeVector3(x, y, z);
}

static void nativeVector3ToJava(JNIEnv *env, jobject jv, b3Vector3 &v) {
  jclass jvclass = NULL;
  jfieldID jxfield = NULL, jyfield = NULL, jzfield = NULL;
  getVector3Fields(env, jvclass, jxfield, jyfield, jzfield);
  env->SetFloatField(jv, jxfield, v.x);
  env->SetFloatField(jv, jyfield, v.y);
  env->SetFloatField(jv, jzfield, v.z);
}

static b3Matrix3x3 javaMatrix3x3ToNative(JNIEnv *env, jobject jv) {
  b3Matrix3x3 v;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/Matrix3x3");
  CHECKFIELD(jm_el_ID, env->GetFieldID(clazz, "m_el", "[Ledu/berkeley/bid/bullet/Vector3;"), "Matrix3x3: Couldnt access native m_el array\n",v)
  jobjectArray jm_el = (jobjectArray)env->GetObjectField(jv, jm_el_ID);
  CHECKVALUE(jm_el, "Matrix3x3: m_el array is null\n", v);
  int i;
  for (i = 0; i < 3; i++) {
    jobject vec3 = env->GetObjectArrayElement(jm_el, i);
    v[i] = javaVector3ToNative(env, vec3);
  }
  return v;
}

static void nativeMatrix3x3ToJava(JNIEnv *env, jobject jv, b3Matrix3x3 &v) {
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/Matrix3x3");
  CHECKFIELD(jm_el_ID, env->GetFieldID(clazz, "m_el", "[Ledu/berkeley/bid/bullet/Vector3;"), "Matrix3x3: Couldnt access native m_el array\n",)
  jobjectArray jm_el = (jobjectArray)env->GetObjectField(jv, jm_el_ID);
  CHECKVALUE(jm_el,"Matrix3x3: m_el array is null\n",)
  int i;
  for (i = 0; i < 3; i++) {
    jobject vec3 = env->GetObjectArrayElement(jm_el, i);
    nativeVector3ToJava(env, vec3, v[i]);
  }
}

static b3Transform javaTransform3ToNative(JNIEnv *env, jobject jv) {
  b3Transform v;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/Transform3");
  CHECKFIELD(jm_basis_ID, env->GetFieldID(clazz, "m_basis", "Ledu/berkeley/bid/bullet/Matrix3x3;"), "Transform3: Couldnt access m_basis\n", v);
  CHECKFIELD(jm_origin_ID, env->GetFieldID(clazz, "m_origin", "Ledu/berkeley/bid/bullet/Vector3;"), "Transform3: Couldnt access m_origin\n", v);
  jobject jm_basis = env->GetObjectField(jv, jm_basis_ID);
  jobject jm_origin = env->GetObjectField(jv, jm_origin_ID);
  CHECKVALUE(jm_basis, "Transform3: m_basis is null\n", v);
  CHECKVALUE(jm_origin, "Transform3: m_origin is null\n", v);
  v.setBasis(javaMatrix3x3ToNative(env, jm_basis));
  v.setOrigin(javaVector3ToNative(env, jm_origin));
  return v;
}

static void nativeTransform3ToJava(JNIEnv *env, jobject jv, b3Transform &v) {
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/Transform3");
  CHECKFIELD(jm_basis_ID, env->GetFieldID(clazz, "m_basis", "Ledu/berkeley/bid/bullet/Matrix3x3;"), "Transform3: Couldnt access m_basis\n",);
  CHECKFIELD(jm_origin_ID, env->GetFieldID(clazz, "m_origin", "Ledu/berkeley/bid/bullet/Vector3;"), "Transform3: Couldnt access m_origin\n",);
  jobject jm_basis = env->GetObjectField(jv, jm_basis_ID);
  jobject jm_origin = env->GetObjectField(jv, jm_origin_ID);
  CHECKVALUE(jm_basis, "Transform3: m_basis is null\n",);
  CHECKVALUE(jm_origin, "Transform3: m_origin is null\n",);
  nativeMatrix3x3ToJava(env, jm_basis, v.getBasis());
  nativeVector3ToJava(env, jm_origin, v.getOrigin());
}

static struct b3JointInfo javaJointInfoToNative(JNIEnv *env, jobject jv) {
  struct b3JointInfo jointInfo;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/JointInfo");
  CHECKFIELD(linkNameID, env->GetFieldID(clazz, "m_linkName", "Ljava/lang/String;"), "JointInfo: can't acccess m_linkName\n", jointInfo);
  CHECKFIELD(jointNameID, env->GetFieldID(clazz, "m_jointName", "Ljava/lang/String;"), "JointInfo: can't acccess m_jointName\n", jointInfo);
  CHECKFIELD(jointTypeID, env->GetFieldID(clazz, "m_jointType", "I"), "JointInfo: can't acccess m_jointType\n", jointInfo);
  CHECKFIELD(qIndexID, env->GetFieldID(clazz, "m_qIndex", "I"), "JointInfo: can't acccess m_qIndex\n", jointInfo);
  CHECKFIELD(uIndexID, env->GetFieldID(clazz, "m_uIndex", "I"), "JointInfo: can't acccess m_uIndex\n", jointInfo);
  CHECKFIELD(jointIndexID, env->GetFieldID(clazz, "m_jointIndex", "I"), "JointInfo: can't acccess m_jointIndex\n", jointInfo);
  CHECKFIELD(flagsID, env->GetFieldID(clazz, "m_flags", "I"), "JointInfo: can't acccess m_flags\n", jointInfo);
  CHECKFIELD(jointDampingID, env->GetFieldID(clazz, "m_jointDamping", "D"), "JointInfo: can't acccess m_jointDamping\n", jointInfo);
  CHECKFIELD(jointFrictionID, env->GetFieldID(clazz, "m_jointFriction", "D"), "JointInfo: can't acccess m_jointFriction\n", jointInfo);
  CHECKFIELD(jointLowerLimitID, env->GetFieldID(clazz, "m_jointLowerLimit", "D"), "JointInfo: can't acccess m_jointLowerLimit\n", jointInfo);
  CHECKFIELD(jointUpperLimitID, env->GetFieldID(clazz, "m_jointUpperLimit", "D"), "JointInfo: can't acccess m_jointUpperLimit\n", jointInfo);
  CHECKFIELD(jointMaxForceID, env->GetFieldID(clazz, "m_jointMaxForce", "D"), "JointInfo: can't acccess m_jointMaxForce\n", jointInfo);
  CHECKFIELD(jointMaxVelocityID, env->GetFieldID(clazz, "m_jointMaxVelocity", "D"), "JointInfo: can't acccess m_jointMaxVelocity\n", jointInfo);
  CHECKFIELD(parentFrameID, env->GetFieldID(clazz, "m_parentFrame", "[D"), "JointInfo: can't acccess m_parentFrame\n", jointInfo);
  CHECKFIELD(childFrameID, env->GetFieldID(clazz, "m_childFrame", "[D"), "JointInfo: can't acccess m_childFrame\n", jointInfo);
  CHECKFIELD(jointAxisID, env->GetFieldID(clazz, "m_jointAxis", "[D"), "JointInfo: can't acccess m_jointAxis\n", jointInfo);
  int i;

  jstring jLinkName = (jstring)env->GetObjectField(jv, linkNameID);
  jstring jJointName = (jstring)env->GetObjectField(jv, jointNameID);
  CHECKVALUE(jLinkName, "JointInfo: m_linkName is null\n", jointInfo);
  CHECKVALUE(jJointName, "JointInfo: m_jointName is null\n", jointInfo);

  char *linkName = (char *)(env->GetStringUTFChars(jLinkName, 0));
  char *jointName = (char *)(env->GetStringUTFChars(jJointName, 0));

  strncpy(jointInfo.m_linkName, linkName, 1024);
  strncpy(jointInfo.m_jointName, jointName, 1024);

  env -> ReleaseStringUTFChars(jJointName, jointName);
  env -> ReleaseStringUTFChars(jLinkName, linkName);

  jointInfo.m_jointType = env->GetIntField(jv, jointTypeID);
  jointInfo.m_qIndex = env->GetIntField(jv, qIndexID);
  jointInfo.m_uIndex = env->GetIntField(jv, uIndexID);
  jointInfo.m_jointIndex = env->GetIntField(jv, jointIndexID);
  jointInfo.m_flags = env->GetIntField(jv, flagsID);
  jointInfo.m_jointDamping = env->GetDoubleField(jv, jointDampingID);
  jointInfo.m_jointFriction = env->GetDoubleField(jv, jointFrictionID);
  jointInfo.m_jointLowerLimit = env->GetDoubleField(jv, jointLowerLimitID);
  jointInfo.m_jointUpperLimit = env->GetDoubleField(jv, jointUpperLimitID);
  jointInfo.m_jointMaxForce = env->GetDoubleField(jv, jointMaxForceID);
  jointInfo.m_jointMaxVelocity = env->GetDoubleField(jv, jointMaxVelocityID);

  jdoubleArray jParentFrame = (jdoubleArray)env->GetObjectField(jv, parentFrameID);
  jdoubleArray jChildFrame = (jdoubleArray)env->GetObjectField(jv, childFrameID);
  jdoubleArray jJointAxis = (jdoubleArray)env->GetObjectField(jv, jointAxisID);
  CHECKVALUE(jParentFrame, "JointInfo: m_parentFrame is null\n", jointInfo);
  CHECKVALUE(jChildFrame, "JointInfo: m_childFrame is null\n", jointInfo);
  CHECKVALUE(jJointAxis, "JointInfo: m_jointAxis is null\n", jointInfo);

  double *parentFrame = (jdouble *)env->GetPrimitiveArrayCritical(jParentFrame, JNI_FALSE);
  double *childFrame = (jdouble *)env->GetPrimitiveArrayCritical(jChildFrame, JNI_FALSE);
  double *jointAxis = (jdouble *)env->GetPrimitiveArrayCritical(jJointAxis, JNI_FALSE);

  for (i = 0; i < 7; i++) {
    jointInfo.m_parentFrame[i] = parentFrame[i];
  }
  for (i = 0; i < 7; i++) {
    jointInfo.m_childFrame[i] = childFrame[i];
  }
  for (i = 0; i < 3; i++) {
    jointInfo.m_jointAxis[i] = jointAxis[i];
  }

  env->ReleasePrimitiveArrayCritical(jJointAxis, jointAxis, 0);
  env->ReleasePrimitiveArrayCritical(jChildFrame, childFrame, 0);
  env->ReleasePrimitiveArrayCritical(jParentFrame, parentFrame, 0);

  return jointInfo;
}

static void nativeJointInfoToJava(JNIEnv *env, jobject jv, struct b3JointInfo &jointInfo) {
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/JointInfo");
  CHECKFIELD(linkNameID, env->GetFieldID(clazz, "m_linkName", "Ljava/lang/String;"), "JointInfo: can't access m_linkName\n",);
  CHECKFIELD(jointNameID, env->GetFieldID(clazz, "m_jointName", "Ljava/lang/String;"), "JointInfo: can't access m_linkName\n",);
  CHECKFIELD(jointTypeID, env->GetFieldID(clazz, "m_jointType", "I"), "JointInfo: can't acccess m_jointType\n",);
  CHECKFIELD(qIndexID, env->GetFieldID(clazz, "m_qIndex", "I"), "JointInfo: can't acccess m_qIndex\n",);
  CHECKFIELD(uIndexID, env->GetFieldID(clazz, "m_uIndex", "I"), "JointInfo: can't acccess m_uIndex\n",);
  CHECKFIELD(jointIndexID, env->GetFieldID(clazz, "m_jointIndex", "I"), "JointInfo: can't acccess m_jointIndex\n",);
  CHECKFIELD(flagsID, env->GetFieldID(clazz, "m_flags", "I"), "JointInfo: can't acccess m_flags\n",);
  CHECKFIELD(jointDampingID, env->GetFieldID(clazz, "m_jointDamping", "D"), "JointInfo: can't acccess m_jointDamping\n",);
  CHECKFIELD(jointFrictionID, env->GetFieldID(clazz, "m_jointFriction", "D"), "JointInfo: can't acccess m_jointFriction\n",);
  CHECKFIELD(jointLowerLimitID, env->GetFieldID(clazz, "m_jointLowerLimit", "D"), "JointInfo: can't acccess m_jointLowerLimit\n",);
  CHECKFIELD(jointUpperLimitID, env->GetFieldID(clazz, "m_jointUpperLimit", "D"), "JointInfo: can't acccess m_jointUpperLimit\n",);
  CHECKFIELD(jointMaxForceID, env->GetFieldID(clazz, "m_jointMaxForce", "D"), "JointInfo: can't acccess m_jointMaxForce\n",);
  CHECKFIELD(jointMaxVelocityID, env->GetFieldID(clazz, "m_jointMaxVelocity", "D"), "JointInfo: can't acccess m_jointMaxVelocity\n",);
  CHECKFIELD(parentFrameID, env->GetFieldID(clazz, "m_parentFrame", "[D"), "JointInfo: can't acccess m_parentFrame\n",);
  CHECKFIELD(childFrameID, env->GetFieldID(clazz, "m_childFrame", "[D"), "JointInfo: can't acccess m_childFrame\n",);
  CHECKFIELD(jointAxisID, env->GetFieldID(clazz, "m_jointAxis", "[D"), "JointInfo: can't acccess m_jointAxis\n",);
  int i;

  jstring jlinkName = env->NewStringUTF(jointInfo.m_linkName);
  jstring jjointName = env->NewStringUTF(jointInfo.m_jointName);
  CHECKVALUE(jlinkName, "JointInfo: m_linkName is null\n",);
  CHECKVALUE(jjointName, "JointInfo: m_jointName is null\n",);
      
  env->SetObjectField(jv, linkNameID, jlinkName);
  env->SetObjectField(jv, jointNameID, jjointName);
  env->SetIntField(jv, jointTypeID, jointInfo.m_jointType);
  env->SetIntField(jv, qIndexID, jointInfo.m_qIndex);
  env->SetIntField(jv, uIndexID, jointInfo.m_uIndex);
  env->SetIntField(jv, jointIndexID, jointInfo.m_jointIndex);
  env->SetIntField(jv, flagsID, jointInfo.m_flags);
  env->SetDoubleField(jv, jointDampingID, jointInfo.m_jointDamping);
  env->SetDoubleField(jv, jointFrictionID, jointInfo.m_jointFriction);
  env->SetDoubleField(jv, jointLowerLimitID, jointInfo.m_jointLowerLimit);
  env->SetDoubleField(jv, jointUpperLimitID, jointInfo.m_jointUpperLimit);
  env->SetDoubleField(jv, jointMaxForceID, jointInfo.m_jointMaxForce);
  env->SetDoubleField(jv, jointMaxVelocityID, jointInfo.m_jointMaxVelocity);

  jdoubleArray jParentFrame = (jdoubleArray)env->GetObjectField(jv, parentFrameID);
  jdoubleArray jChildFrame = (jdoubleArray)env->GetObjectField(jv, childFrameID);
  jdoubleArray jJointAxis = (jdoubleArray)env->GetObjectField(jv, jointAxisID);
  CHECKVALUE(jParentFrame, "JointInfo: m_parentFrame is null\n",);
  CHECKVALUE(jChildFrame, "JointInfo: m_childFrame is null\n",);
  CHECKVALUE(jJointAxis, "JointInfo: m_jointAxis is null\n",);

  double *parentFrame = (jdouble *)env->GetPrimitiveArrayCritical(jParentFrame, JNI_FALSE);
  double *childFrame = (jdouble *)env->GetPrimitiveArrayCritical(jChildFrame, JNI_FALSE);
  double *jointAxis = (jdouble *)env->GetPrimitiveArrayCritical(jJointAxis, JNI_FALSE);

  for (i = 0; i < 7; i++) {
    parentFrame[i] = jointInfo.m_parentFrame[i];
  }
  for (i = 0; i < 7; i++) {
    childFrame[i] = jointInfo.m_childFrame[i];
  }
  for (i = 0; i < 3; i++) {
    jointAxis[i] = jointInfo.m_jointAxis[i];
  }

  env->ReleasePrimitiveArrayCritical(jJointAxis, jointAxis, 0);
  env->ReleasePrimitiveArrayCritical(jChildFrame, childFrame, 0);
  env->ReleasePrimitiveArrayCritical(jParentFrame, parentFrame, 0);
}

static struct b3JointSensorState javaJointSensorStateToNative(JNIEnv *env, jobject jv) {
  struct b3JointSensorState jointSensorState;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/JointSensorState");
  CHECKFIELD(jointPositionID, env->GetFieldID(clazz, "m_jointPosition", "D"), "JointSensorState: can't acccess m_jointPosition\n", jointSensorState;);
  CHECKFIELD(jointVelocityID, env->GetFieldID(clazz, "m_jointVelocity", "D"), "JointSensorState: can't acccess m_jointVelocity\n", jointSensorState;);
  CHECKFIELD(jointMotorTorqueID, env->GetFieldID(clazz, "m_jointMotorTorque", "D"), "JointSensorState: can't acccess m_jointMotorTorque\n", jointSensorState;);
  CHECKFIELD(jointForceTorqueID, env->GetFieldID(clazz, "m_jointForceTorque", "[D"), "JointSensorState: can't acccess m_jointForceTorque\n", jointSensorState;);
  int i;

  jointSensorState.m_jointPosition = env->GetDoubleField(jv, jointPositionID);
  jointSensorState.m_jointVelocity = env->GetDoubleField(jv, jointVelocityID);
  jointSensorState.m_jointMotorTorque = env->GetDoubleField(jv, jointMotorTorqueID);

  jdoubleArray jJointForceTorque = (jdoubleArray)env->GetObjectField(jv, jointForceTorqueID);
  CHECKVALUE(jJointForceTorque, "JointSensorState: m_jointForceTorque is null\n", jointSensorState);
  
  double *jointForceTorque = (jdouble *)env->GetPrimitiveArrayCritical(jJointForceTorque, JNI_FALSE);

  for (i = 0; i < 6; i++) {
    jointSensorState.m_jointForceTorque[i] = jointForceTorque[i];
  }

  env->ReleasePrimitiveArrayCritical(jJointForceTorque, jointForceTorque, 0);
  return jointSensorState;
}

static void nativeJointSensorStateToJava(JNIEnv *env, jobject jv, struct b3JointSensorState &jointSensorState) {
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/JointSensorState");
  CHECKFIELD(jointPositionID, env->GetFieldID(clazz, "m_jointPosition", "D"), "JointSensorState: can't acccess m_jointPosition\n",);
  CHECKFIELD(jointVelocityID, env->GetFieldID(clazz, "m_jointVelocity", "D"), "JointSensorState: can't acccess m_jointVelocity\n",);
  CHECKFIELD(jointMotorTorqueID, env->GetFieldID(clazz, "m_jointMotorTorque", "D"), "JointSensorState: can't acccess m_jointMotorTorque\n",);
  CHECKFIELD(jointForceTorqueID, env->GetFieldID(clazz, "m_jointForceTorque", "[D"), "JointSensorState: can't acccess m_jointForceTorque\n",);
  int i;

  env->SetDoubleField(jv, jointPositionID, jointSensorState.m_jointPosition);
  env->SetDoubleField(jv, jointVelocityID, jointSensorState.m_jointVelocity);
  env->SetDoubleField(jv, jointMotorTorqueID, jointSensorState.m_jointMotorTorque);

  jdoubleArray jJointForceTorque = (jdoubleArray)env->GetObjectField(jv, jointForceTorqueID);
  CHECKVALUE(jJointForceTorque, "JointSensorState: m_jointForceTorque is null\n",);

  double *jointForceTorque = (jdouble *)env->GetPrimitiveArrayCritical(jJointForceTorque, JNI_FALSE);

  for (i = 0; i < 6; i++) {
    jointForceTorque[i] = jointSensorState.m_jointForceTorque[i];
  }

  env->ReleasePrimitiveArrayCritical(jJointForceTorque, jointForceTorque, 0);
}

static struct b3JointStates2 javaJointStates2ToNative(JNIEnv *env, jobject jv, int numJoints) {
  int i;
  struct b3JointStates2 jointStates2;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/JointStates2");
  CHECKFIELD(bodyUniqueID, env->GetFieldID(clazz, "m_bodyUniqueId", "I"), "JointStates2: can't acccess m_bodyUniqueId\n", jointStates2);
  CHECKFIELD(numDegreeOfFreedomQID, env->GetFieldID(clazz, "m_numDegreeOfFreedomQ", "I"), "JointStates2: can't acccess m_numDegreeOfFreedomQ\n", jointStates2);
  CHECKFIELD(numDegreeOfFreedomUID, env->GetFieldID(clazz, "m_numDegreeOfFreedomU", "I"), "JointStates2: can't acccess m_numDegreeOfFreedomU\n", jointStates2);
  CHECKFIELD(rootLocalInertialFrameID, env->GetFieldID(clazz, "m_rootLocalInertialFrame", "Ledu/berkeley/bid/bullet/Transform3;"),
	     "JointStates2: can't access m_rootLocalInertialFrame\n", jointStates2);
  CHECKFIELD(actualStateQID, env->GetFieldID(clazz, "m_actualStateQ", "[D"), "JointStates2: can't acccess m_actualStateQ\n", jointStates2);
  CHECKFIELD(actualStateQdotID, env->GetFieldID(clazz, "m_actualStateQdot", "[D"), "JointStates2: can't acccess m_actualStateQdot\n", jointStates2);
  CHECKFIELD(jointReactionForcesID, env->GetFieldID(clazz, "m_jointReactionForces", "[D"), "JointStates2: can't acccess m_jointReactionForces\n", jointStates2);

  jointStates2.m_bodyUniqueId = env->GetIntField(jv, bodyUniqueID);
  jointStates2.m_numDegreeOfFreedomQ = env->GetIntField(jv, numDegreeOfFreedomQID);
  jointStates2.m_numDegreeOfFreedomU = env->GetIntField(jv, numDegreeOfFreedomUID);

  jobject jrootLocalInertialFrame = env->GetObjectField(jv, rootLocalInertialFrameID);
  CHECKVALUE(jrootLocalInertialFrame, "JointStates2: m_rootLocalInertialFrame is null\n", jointStates2);
  jointStates2.m_rootLocalInertialFrame = javaTransform3ToNative(env, jrootLocalInertialFrame);

  jdoubleArray jactualStateQ = (jdoubleArray)env->GetObjectField(jv, actualStateQID);
  jdoubleArray jactualStateQdot = (jdoubleArray)env->GetObjectField(jv, actualStateQdotID);
  jdoubleArray jjointReactionForces = (jdoubleArray)env->GetObjectField(jv, jointReactionForcesID);
  CHECKVALUE(jactualStateQ, "JointStates2: m_actualStateQ is null\n", jointStates2);
  CHECKVALUE(jactualStateQdot, "JointStates2: m_actualStateQdot is null\n", jointStates2);
  CHECKVALUE(jjointReactionForces, "JointStates2: m_jointReactionForces is null\n", jointStates2);

  double *actualStateQ = (jdouble *)env->GetPrimitiveArrayCritical(jactualStateQ, JNI_FALSE);
  double *actualStateQdot = (jdouble *)env->GetPrimitiveArrayCritical(jactualStateQdot, JNI_FALSE);
  double *jointReactionForces = (jdouble *)env->GetPrimitiveArrayCritical(jjointReactionForces, JNI_FALSE);

  jointStates2.m_actualStateQ.resize(jointStates2.m_numDegreeOfFreedomQ);
  jointStates2.m_actualStateQdot.resize(jointStates2.m_numDegreeOfFreedomU);
  jointStates2.m_jointReactionForces.resize(numJoints*6);

  for (i = 0; i < jointStates2.m_numDegreeOfFreedomQ; i++) {
    jointStates2.m_actualStateQ[i] = actualStateQ[i];
  }
  for (i = 0; i < jointStates2.m_numDegreeOfFreedomU; i++) {
    jointStates2.m_actualStateQdot[i] = actualStateQdot[i];
  }
  for (i = 0; i < numJoints*6; i++) {
    jointStates2.m_jointReactionForces[i] = jointReactionForces[i];
  }

  env->ReleasePrimitiveArrayCritical(jjointReactionForces, jointReactionForces, 0);
  env->ReleasePrimitiveArrayCritical(jactualStateQdot, actualStateQdot, 0);
  env->ReleasePrimitiveArrayCritical(jactualStateQ, actualStateQ, 0);
  return jointStates2;
}

static void nativeJointStates2ToJava(JNIEnv *env, jobject jv, struct b3JointStates2 &jointStates2, int numJoints) {
  int i;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/JointStates2");
  CHECKFIELD(bodyUniqueID, env->GetFieldID(clazz, "m_bodyUniqueId", "I"), "JointStates2: can't acccess m_bodyUniqueId\n",);
  CHECKFIELD(numDegreeOfFreedomQID, env->GetFieldID(clazz, "m_numDegreeOfFreedomQ", "I"), "JointStates2: can't acccess m_numDegreeOfFreedomQ\n",);
  CHECKFIELD(numDegreeOfFreedomUID, env->GetFieldID(clazz, "m_numDegreeOfFreedomU", "I"), "JointStates2: can't acccess m_numDegreeOfFreedomU\n",);
  CHECKFIELD(rootLocalInertialFrameID, env->GetFieldID(clazz, "m_rootLocalInertialFrame", "Ledu/berkeley/bid/bullet/Transform3;"),
	     "JointStates2: can't acccess m_rootLocalInertialFrame\n",);
  CHECKFIELD(actualStateQID, env->GetFieldID(clazz, "m_actualStateQ", "[D"), "JointStates2: can't acccess m_actualStateQ\n",);
  CHECKFIELD(actualStateQdotID, env->GetFieldID(clazz, "m_actualStateQdot", "[D"), "JointStates2: can't acccess m_actualStateQdot\n",);
  CHECKFIELD(jointReactionForcesID, env->GetFieldID(clazz, "m_jointReactionForces", "[D"), "JointStates2: can't acccess m_jointReactionForces\n",);

  env->SetIntField(jv, bodyUniqueID, jointStates2.m_bodyUniqueId);
  env->SetIntField(jv, numDegreeOfFreedomQID, jointStates2.m_numDegreeOfFreedomQ);
  env->SetIntField(jv, numDegreeOfFreedomUID, jointStates2.m_numDegreeOfFreedomU);

  jobject jrootLocalInertialFrame = env->GetObjectField(jv, rootLocalInertialFrameID);
  CHECKVALUE(jrootLocalInertialFrame, "JointStates2: m_rootLocalInertialFrame is null\n",);
  nativeTransform3ToJava(env, jrootLocalInertialFrame, jointStates2.m_rootLocalInertialFrame);

  jdoubleArray jactualStateQ = env->NewDoubleArray(jointStates2.m_numDegreeOfFreedomQ);
  jdoubleArray jactualStateQdot = env->NewDoubleArray(jointStates2.m_numDegreeOfFreedomU);
  jdoubleArray jjointReactionForces = env->NewDoubleArray(numJoints*6);

  env->SetObjectField(jv, actualStateQID, jactualStateQ);
  env->SetObjectField(jv, actualStateQdotID, jactualStateQdot);
  env->SetObjectField(jv, jointReactionForcesID, jjointReactionForces);

  double *actualStateQ = (jdouble *)env->GetPrimitiveArrayCritical(jactualStateQ, JNI_FALSE);
  double *actualStateQdot = (jdouble *)env->GetPrimitiveArrayCritical(jactualStateQdot, JNI_FALSE);
  double *jointReactionForces = (jdouble *)env->GetPrimitiveArrayCritical(jjointReactionForces, JNI_FALSE);

  for (i = 0; i < jointStates2.m_numDegreeOfFreedomQ; i++) {
    actualStateQ[i] = jointStates2.m_actualStateQ[i];
  }
  for (i = 0; i < jointStates2.m_numDegreeOfFreedomU; i++) {
    actualStateQdot[i] = jointStates2.m_actualStateQdot[i];
  }
  for (i = 0; i < numJoints*6; i++) {
    jointReactionForces[i] = jointStates2.m_jointReactionForces[i];
  }

  env->ReleasePrimitiveArrayCritical(jjointReactionForces, jointReactionForces, 0);
  env->ReleasePrimitiveArrayCritical(jactualStateQdot, actualStateQdot, 0);
  env->ReleasePrimitiveArrayCritical(jactualStateQ, actualStateQ, 0);  
}

static struct b3RobotSimulatorJointMotorArgs javaJointMotorArgsToNative(JNIEnv *env, jobject jv) {
  struct b3RobotSimulatorJointMotorArgs motorArgs(0);
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/JointMotorArgs");
  CHECKFIELD(controlModeID, env->GetFieldID(clazz, "m_controlMode", "I"), "JointMotorArgs: can't acccess m_controlMode\n", motorArgs);
  CHECKFIELD(targetPositionID, env->GetFieldID(clazz, "m_targetPosition", "D"), "JointMotorArgs: can't acccess m_targetPosition\n", motorArgs);
  CHECKFIELD(kpID, env->GetFieldID(clazz, "m_kp", "D"), "JointMotorArgs: can't acccess m_kp\n", motorArgs);
  CHECKFIELD(targetVelocityID, env->GetFieldID(clazz, "m_targetVelocity", "D"), "JointMotorArgs: can't acccess m_targetVelocity\n", motorArgs);
  CHECKFIELD(kdID, env->GetFieldID(clazz, "m_kd", "D"), "JointMotorArgs: can't acccess m_kd\n", motorArgs);
  CHECKFIELD(maxTorqueValueID, env->GetFieldID(clazz, "m_maxTorqueValue", "D"), "JointMotorArgs: can't acccess m_maxTorqueValue\n", motorArgs);

  motorArgs.m_controlMode = env->GetIntField(jv, controlModeID);
  motorArgs.m_targetPosition = env->GetDoubleField(jv, targetPositionID);
  motorArgs.m_kp = env->GetDoubleField(jv, kpID);
  motorArgs.m_targetVelocity = env->GetDoubleField(jv, targetVelocityID);
  motorArgs.m_kd = env->GetDoubleField(jv, kdID);
  motorArgs.m_maxTorqueValue = env->GetDoubleField(jv, maxTorqueValueID);

  return motorArgs;
}

static void nativeJointMotorArgsToJava(JNIEnv *env, jobject jv, struct b3RobotSimulatorJointMotorArgs &motorArgs) {
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/JointMotorArgs");
  CHECKFIELD(controlModeID, env->GetFieldID(clazz, "m_controlMode", "I"), "JointMotorArgs: can't acccess m_controlMode\n",);
  CHECKFIELD(targetPositionID, env->GetFieldID(clazz, "m_targetPosition", "D"), "JointMotorArgs: can't acccess m_targetPosition\n",);
  CHECKFIELD(kpID, env->GetFieldID(clazz, "m_kp", "D"), "JointMotorArgs: can't acccess m_kp\n",);
  CHECKFIELD(targetVelocityID, env->GetFieldID(clazz, "m_targetVelocity", "D"), "JointMotorArgs: can't acccess m_targetVelocity\n",);
  CHECKFIELD(kdID, env->GetFieldID(clazz, "m_kd", "D"), "JointMotorArgs: can't acccess m_kd\n",);
  CHECKFIELD(maxTorqueValueID, env->GetFieldID(clazz, "m_maxTorqueValue", "D"), "JointMotorArgs: can't acccess m_maxTorqueValue\n",);

  env->SetIntField(jv, controlModeID, motorArgs.m_controlMode);
  env->SetDoubleField(jv, targetPositionID, motorArgs.m_targetPosition);
  env->SetDoubleField(jv, kpID, motorArgs.m_kp);
  env->SetDoubleField(jv, targetVelocityID, motorArgs.m_targetVelocity);
  env->SetDoubleField(jv, kdID, motorArgs.m_kd);
  env->SetDoubleField(jv, maxTorqueValueID, motorArgs.m_maxTorqueValue);

}

static struct b3RobotSimulatorInverseKinematicArgs javaInverseKinematicArgsToNative(JNIEnv *env, jobject jv) {
  int n, i;
  struct b3RobotSimulatorInverseKinematicArgs args;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/InverseKinematicArgs");
  CHECKFIELD(bodyUniqueIdID, env->GetFieldID(clazz, "m_bodyUniqueId", "I"), "InverseKinematicArgs: can't access m_bodyUniqueId\n", args);
  CHECKFIELD(endEffectorLinkIndexID, env->GetFieldID(clazz, "m_endEffectorLinkIndex", "I"), "InverseKinematicArgs: can't access m_endEffectorLinkIndex\n", args);
  CHECKFIELD(flagsID, env->GetFieldID(clazz, "m_flags", "I"), "InverseKinematicArgs: can't access m_flags\n", args);
  CHECKFIELD(numDegreeOfFreedomID, env->GetFieldID(clazz, "m_numDegreeOfFreedom", "I"), "InverseKinematicArgs: can't access m_numDegreeOfFreedom\n", args);
  CHECKFIELD(endEffectorTargetPositionID, env->GetFieldID(clazz, "m_endEffectorTargetPosition", "[D"), "InverseKinematicArgs: can't access m_endEffectorTargetPosition\n", args);
  CHECKFIELD(endEffectorTargetOrientationID, env->GetFieldID(clazz, "m_endEffectorTargetOrientation", "[D"), "InverseKinematicArgs: can't access m_endEffectorTargetOrientation\n", args);
  CHECKFIELD(lowerLimitsID, env->GetFieldID(clazz, "m_lowerLimits", "[D"), "InverseKinematicArgs: can't access m_lowerLimits\n", args);
  CHECKFIELD(upperLimitsID, env->GetFieldID(clazz, "m_upperLimits", "[D"), "InverseKinematicArgs: can't access m_upperLimits\n", args);
  CHECKFIELD(jointRangesID, env->GetFieldID(clazz, "m_jointRanges", "[D"), "InverseKinematicArgs: can't access m_jointRanges\n", args);
  CHECKFIELD(restPosesID, env->GetFieldID(clazz, "m_restPoses", "[D"), "InverseKinematicArgs: can't access m_restPoses\n", args);
  CHECKFIELD(jointDampingID, env->GetFieldID(clazz, "m_jointDamping", "[D"), "InverseKinematicArgs: can't access m_jointDamping\n", args);

  args.m_bodyUniqueId = env->GetIntField(jv, bodyUniqueIdID);
  args.m_endEffectorLinkIndex = env->GetIntField(jv, endEffectorLinkIndexID);
  args.m_flags = env->GetIntField(jv, flagsID);
  args.m_numDegreeOfFreedom = env->GetIntField(jv, numDegreeOfFreedomID);
  jdoubleArray jendEffectorTargetPosition = (jdoubleArray)env->GetObjectField(jv, endEffectorTargetPositionID);
  jdoubleArray jendEffectorTargetOrientation = (jdoubleArray)env->GetObjectField(jv, endEffectorTargetOrientationID);
  jdoubleArray jlowerLimits = (jdoubleArray)env->GetObjectField(jv, lowerLimitsID);
  jdoubleArray jupperLimits = (jdoubleArray)env->GetObjectField(jv, upperLimitsID);
  jdoubleArray jjointRanges = (jdoubleArray)env->GetObjectField(jv, jointRangesID);
  jdoubleArray jrestPoses = (jdoubleArray)env->GetObjectField(jv, restPosesID);
  jdoubleArray jjointDamping = (jdoubleArray)env->GetObjectField(jv, jointDampingID);
  CHECKVALUE(jendEffectorTargetPosition, "InverseKinematicArgs: can't acccess m_endEffectorTargetPosition\n", args);
  CHECKVALUE(jendEffectorTargetOrientation, "InverseKinematicArgs: can't acccess m_endEffectorTargetOrientation\n", args);
  CHECKVALUE(jlowerLimits, "InverseKinematicArgs: can't acccess m_lowerLimits\n", args);
  CHECKVALUE(jupperLimits, "InverseKinematicArgs: can't acccess m_upperLimits\n", args);
  CHECKVALUE(jjointRanges, "InverseKinematicArgs: can't acccess m_jointRanges\n", args);
  CHECKVALUE(jrestPoses, "InverseKinematicArgs: can't acccess m_restPoses\n", args);
  CHECKVALUE(jjointDamping, "InverseKinematicArgs: can't acccess m_jointDamping\n", args);

  double *endEffectorTargetPosition = (jdouble *)env->GetPrimitiveArrayCritical(jendEffectorTargetPosition, JNI_FALSE);
  double *endEffectorTargetOrientation = (jdouble *)env->GetPrimitiveArrayCritical(jendEffectorTargetOrientation, JNI_FALSE);
  double *lowerLimits = (jdouble *)env->GetPrimitiveArrayCritical(jlowerLimits, JNI_FALSE);
  double *upperLimits = (jdouble *)env->GetPrimitiveArrayCritical(jupperLimits, JNI_FALSE);
  double *jointRanges = (jdouble *)env->GetPrimitiveArrayCritical(jjointRanges, JNI_FALSE);
  double *restPoses = (jdouble *)env->GetPrimitiveArrayCritical(jrestPoses, JNI_FALSE);
  double *jointDamping = (jdouble *)env->GetPrimitiveArrayCritical(jjointDamping, JNI_FALSE);

  args.m_endEffectorTargetPosition[0] = endEffectorTargetPosition[0];
  args.m_endEffectorTargetPosition[1] = endEffectorTargetPosition[1];
  args.m_endEffectorTargetPosition[2] = endEffectorTargetPosition[2];

  args.m_endEffectorTargetOrientation[0] = endEffectorTargetOrientation[0];
  args.m_endEffectorTargetOrientation[1] = endEffectorTargetOrientation[1];
  args.m_endEffectorTargetOrientation[2] = endEffectorTargetOrientation[2];
  args.m_endEffectorTargetOrientation[3] = endEffectorTargetOrientation[3];

  n = env->GetArrayLength(jlowerLimits);
  args.m_lowerLimits.resize(n);
  for (i = 0; i < n; i++) {
    args.m_lowerLimits[i] = lowerLimits[i];
  }

  n = env->GetArrayLength(jupperLimits);
  args.m_upperLimits.resize(n);
  for (i = 0; i < n; i++) {
    args.m_upperLimits[i] = upperLimits[i];
  }

  n = env->GetArrayLength(jjointRanges);
  args.m_jointRanges.resize(n);
  for (i = 0; i < n; i++) {
    args.m_jointRanges[i] = jointRanges[i];
  }

  n = env->GetArrayLength(jrestPoses);
  args.m_restPoses.resize(n);
  for (i = 0; i < n; i++) {
    args.m_restPoses[i] = restPoses[i];
  }

  n = env->GetArrayLength(jjointDamping);
  args.m_jointDamping.resize(n);
  for (i = 0; i < n; i++) {
    args.m_jointDamping[i] = jointDamping[i];
  }

  env->ReleasePrimitiveArrayCritical(jjointDamping, jointDamping, 0);
  env->ReleasePrimitiveArrayCritical(jjointRanges, jointRanges, 0);
  env->ReleasePrimitiveArrayCritical(jrestPoses, restPoses, 0);
  env->ReleasePrimitiveArrayCritical(jupperLimits, upperLimits, 0);
  env->ReleasePrimitiveArrayCritical(jlowerLimits, lowerLimits, 0);
  env->ReleasePrimitiveArrayCritical(jendEffectorTargetOrientation, endEffectorTargetOrientation, 0);
  env->ReleasePrimitiveArrayCritical(jendEffectorTargetPosition, endEffectorTargetPosition, 0);

  return args;
}

static void nativeInverseKinematicsResultsToJava(JNIEnv *env, jobject jv, struct b3RobotSimulatorInverseKinematicsResults &results) {
  int i, n;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/InverseKinematicsResults");
  CHECKFIELD(bodyUniqueIdID, env->GetFieldID(clazz, "m_bodyUniqueId", "I"), "InverseKinematicsResults: can't access m_bodyUniqueId\n",);
  CHECKFIELD(calculatedJointPositionsID, env->GetFieldID(clazz, "m_calculatedJointPositions", "[D"), "InverseKinematicsResults: can't access m_calculatedJointPositions\n",);
  env->SetIntField(jv, bodyUniqueIdID, results.m_bodyUniqueId);
  n = results.m_calculatedJointPositions.size();
  jdoubleArray jcalculatedJointPositions = env->NewDoubleArray(n);
  env->SetObjectField(jv, calculatedJointPositionsID, jcalculatedJointPositions);
  jdouble *calculatedJointPositions = (jdouble *)env->GetPrimitiveArrayCritical(jcalculatedJointPositions, JNI_FALSE);
  for (i = 0; i < n; i++) {
    calculatedJointPositions[i] = results.m_calculatedJointPositions[i];
  }
  env->ReleasePrimitiveArrayCritical(jcalculatedJointPositions, calculatedJointPositions, 0);
}

static struct b3LinkState javaLinkStateToNative(JNIEnv *env, jobject jv) {
  int n, i;
  struct b3LinkState linkState;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/LinkState");
  CHECKFIELD(worldPositionID, env->GetFieldID(clazz, "m_worldPosition", "[D"), "LinkState: can't access m_worldPosition\n", linkState);
  CHECKFIELD(worldOrientationID, env->GetFieldID(clazz, "m_worldOrientation", "[D"), "LinkState: can't access m_worldOrientation\n", linkState);
  CHECKFIELD(localInertialPositionID, env->GetFieldID(clazz, "m_localInertialPosition", "[D"), "LinkState: can't access m_localInertialPosition\n", linkState);
  CHECKFIELD(localInertialOrientationID, env->GetFieldID(clazz, "m_localInertialOrientation", "[D"), "LinkState: can't access m_localInertialOrientation\n", linkState);
  CHECKFIELD(worldLinkFramePositionID, env->GetFieldID(clazz, "m_worldLinkFramePosition", "[D"), "LinkState: can't access m_worldLinkFramePosition\n", linkState);
  CHECKFIELD(worldLinkFrameOrientationID, env->GetFieldID(clazz, "m_worldLinkFrameOrientation", "[D"), "LinkState: can't access m_worldLinkFrameOrientation\n", linkState);
  CHECKFIELD(worldLinearVelocityID, env->GetFieldID(clazz, "m_worldLinearVelocity", "[D"), "LinkState: can't access m_worldLinearVelocity\n", linkState);
  CHECKFIELD(worldAngularVelocityID, env->GetFieldID(clazz, "m_worldAngularVelocity", "[D"), "LinkState: can't access m_worldAngularVelocity\n", linkState);
  CHECKFIELD(worldAABBMinID, env->GetFieldID(clazz, "m_worldAABBMin", "[D"), "LinkState: can't access m_worldAABBMin\n", linkState);
  CHECKFIELD(worldAABBMaxID, env->GetFieldID(clazz, "m_worldAABBMax", "[D"), "LinkState: can't access m_worldAABBMax\n", linkState);

  jdoubleArray jworldPosition = (jdoubleArray)env->GetObjectField(jv, worldPositionID);
  jdoubleArray jworldOrientation = (jdoubleArray)env->GetObjectField(jv, worldOrientationID);
  jdoubleArray jlocalInertialPosition = (jdoubleArray)env->GetObjectField(jv, localInertialPositionID);
  jdoubleArray jlocalInertialOrientation = (jdoubleArray)env->GetObjectField(jv, localInertialOrientationID);
  jdoubleArray jworldLinkFramePosition = (jdoubleArray)env->GetObjectField(jv, worldLinkFramePositionID);
  jdoubleArray jworldLinkFrameOrientation = (jdoubleArray)env->GetObjectField(jv, worldLinkFrameOrientationID);
  jdoubleArray jworldLinearVelocity = (jdoubleArray)env->GetObjectField(jv, worldLinearVelocityID);
  jdoubleArray jworldAngularVelocity = (jdoubleArray)env->GetObjectField(jv, worldAngularVelocityID);
  jdoubleArray jworldAABBMin = (jdoubleArray)env->GetObjectField(jv, worldAABBMinID);
  jdoubleArray jworldAABBMax = (jdoubleArray)env->GetObjectField(jv, worldAABBMaxID);

  CHECKVALUE(jworldPosition, "LinkState: worldPosition is null", linkState);
  CHECKVALUE(jworldOrientation, "LinkState: worldOrientation is null", linkState);
  CHECKVALUE(jlocalInertialPosition, "LinkState: localInertialPosition is null", linkState); 
  CHECKVALUE(jlocalInertialOrientation, "LinkState: localInertialOrientation is null", linkState);
  CHECKVALUE(jworldLinkFramePosition, "LinkState: worldLinkFramePosition is null", linkState);
  CHECKVALUE(jworldLinkFrameOrientation, "LinkState: worldLinkFrameOrientation is null", linkState);
  CHECKVALUE(jworldLinearVelocity, "LinkState: worldLinearVelocity is null", linkState); 
  CHECKVALUE(jworldAngularVelocity, "LinkState: worldAngularVelocity is null", linkState);
  CHECKVALUE(jworldAABBMin, "LinkState: worldAABBMin is null", linkState);
  CHECKVALUE(jworldAABBMax, "LinkState: worldAABBMax is null", linkState);

  double *worldPosition = env->GetDoubleArrayElements(jworldPosition, NULL);
  double *worldOrientation = env->GetDoubleArrayElements(jworldOrientation, NULL);
  double *localInertialPosition = env->GetDoubleArrayElements(jlocalInertialPosition, NULL);
  double *localInertialOrientation = env->GetDoubleArrayElements(jlocalInertialOrientation, NULL);
  double *worldLinkFramePosition = env->GetDoubleArrayElements(jworldLinkFramePosition, NULL);
  double *worldLinkFrameOrientation = env->GetDoubleArrayElements(jworldLinkFrameOrientation, NULL);
  double *worldLinearVelocity = env->GetDoubleArrayElements(jworldLinearVelocity, NULL);
  double *worldAngularVelocity = env->GetDoubleArrayElements(jworldAngularVelocity, NULL);
  double *worldAABBMin = env->GetDoubleArrayElements(jworldAABBMin, NULL);
  double *worldAABBMax = env->GetDoubleArrayElements(jworldAABBMax, NULL);

  linkState.m_worldPosition[0] = worldPosition[0];
  linkState.m_worldPosition[1] = worldPosition[1];
  linkState.m_worldPosition[2] = worldPosition[2];
  linkState.m_worldOrientation[0] = worldOrientation[0];
  linkState.m_worldOrientation[1] = worldOrientation[1];
  linkState.m_worldOrientation[2] = worldOrientation[2];
  linkState.m_worldOrientation[3] = worldOrientation[3];

  linkState.m_localInertialPosition[0] = localInertialPosition[0];
  linkState.m_localInertialPosition[1] = localInertialPosition[1];
  linkState.m_localInertialPosition[2] = localInertialPosition[2];
  linkState.m_localInertialOrientation[0] = localInertialOrientation[0];
  linkState.m_localInertialOrientation[1] = localInertialOrientation[1];
  linkState.m_localInertialOrientation[2] = localInertialOrientation[2];
  linkState.m_localInertialOrientation[3] = localInertialOrientation[3];

  linkState.m_worldLinkFramePosition[0] = worldLinkFramePosition[0];
  linkState.m_worldLinkFramePosition[1] = worldLinkFramePosition[1];
  linkState.m_worldLinkFramePosition[2] = worldLinkFramePosition[2];
  linkState.m_worldLinkFrameOrientation[0] = worldLinkFrameOrientation[0];
  linkState.m_worldLinkFrameOrientation[1] = worldLinkFrameOrientation[1];
  linkState.m_worldLinkFrameOrientation[2] = worldLinkFrameOrientation[2];
  linkState.m_worldLinkFrameOrientation[3] = worldLinkFrameOrientation[3];

  linkState.m_worldLinearVelocity[0] = worldLinearVelocity[0];
  linkState.m_worldLinearVelocity[1] = worldLinearVelocity[1];
  linkState.m_worldLinearVelocity[2] = worldLinearVelocity[2];
  linkState.m_worldAngularVelocity[0] = worldAngularVelocity[0];
  linkState.m_worldAngularVelocity[1] = worldAngularVelocity[1];
  linkState.m_worldAngularVelocity[2] = worldAngularVelocity[2];

  linkState.m_worldAABBMin[0] = worldAABBMin[0];
  linkState.m_worldAABBMin[1] = worldAABBMin[1];
  linkState.m_worldAABBMin[2] = worldAABBMin[2];
  linkState.m_worldAABBMax[0] = worldAABBMax[0];
  linkState.m_worldAABBMax[1] = worldAABBMax[1];
  linkState.m_worldAABBMax[2] = worldAABBMax[2];

  env->ReleaseDoubleArrayElements(jworldAABBMax, worldAABBMax, 0);
  env->ReleaseDoubleArrayElements(jworldAABBMin, worldAABBMin, 0);
  env->ReleaseDoubleArrayElements(jworldAngularVelocity, worldAngularVelocity, 0);
  env->ReleaseDoubleArrayElements(jworldLinearVelocity, worldLinearVelocity, 0);
  env->ReleaseDoubleArrayElements(jworldLinkFrameOrientation, worldLinkFrameOrientation, 0);
  env->ReleaseDoubleArrayElements(jworldLinkFramePosition, worldLinkFramePosition, 0);
  env->ReleaseDoubleArrayElements(jlocalInertialOrientation, localInertialOrientation, 0);
  env->ReleaseDoubleArrayElements(jlocalInertialPosition, localInertialPosition, 0);
  env->ReleaseDoubleArrayElements(jworldOrientation, worldOrientation, 0);
  env->ReleaseDoubleArrayElements(jworldPosition, worldPosition, 0);
  
  return linkState;
}

static void nativeLinkStateToJava(JNIEnv *env, jobject jv, struct b3LinkState &linkState) {
  int n, i;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/LinkState");
  CHECKFIELD(worldPositionID, env->GetFieldID(clazz, "m_worldPosition", "[D"), "LinkState: can't access m_worldPosition\n",);
  CHECKFIELD(worldOrientationID, env->GetFieldID(clazz, "m_worldOrientation", "[D"), "LinkState: can't access m_worldOrientation\n",);
  CHECKFIELD(localInertialPositionID, env->GetFieldID(clazz, "m_localInertialPosition", "[D"), "LinkState: can't access m_localInertialPosition\n",);
  CHECKFIELD(localInertialOrientationID, env->GetFieldID(clazz, "m_localInertialOrientation", "[D"), "LinkState: can't access m_localInertialOrientation\n",);
  CHECKFIELD(worldLinkFramePositionID, env->GetFieldID(clazz, "m_worldLinkFramePosition", "[D"), "LinkState: can't access m_worldLinkFramePosition\n",);
  CHECKFIELD(worldLinkFrameOrientationID, env->GetFieldID(clazz, "m_worldLinkFrameOrientation", "[D"), "LinkState: can't access m_worldLinkFrameOrientation\n",);
  CHECKFIELD(worldLinearVelocityID, env->GetFieldID(clazz, "m_worldLinearVelocity", "[D"), "LinkState: can't access m_worldLinearVelocity\n",);
  CHECKFIELD(worldAngularVelocityID, env->GetFieldID(clazz, "m_worldAngularVelocity", "[D"), "LinkState: can't access m_worldAngularVelocity\n",);
  CHECKFIELD(worldAABBMinID, env->GetFieldID(clazz, "m_worldAABBMin", "[D"), "LinkState: can't access m_worldAABBMin\n",);
  CHECKFIELD(worldAABBMaxID, env->GetFieldID(clazz, "m_worldAABBMax", "[D"), "LinkState: can't access m_worldAABBMax\n",);

  jdoubleArray jworldPosition = (jdoubleArray)env->GetObjectField(jv, worldPositionID);
  jdoubleArray jworldOrientation = (jdoubleArray)env->GetObjectField(jv, worldOrientationID);
  jdoubleArray jlocalInertialPosition = (jdoubleArray)env->GetObjectField(jv, localInertialPositionID);
  jdoubleArray jlocalInertialOrientation = (jdoubleArray)env->GetObjectField(jv, localInertialOrientationID);
  jdoubleArray jworldLinkFramePosition = (jdoubleArray)env->GetObjectField(jv, worldLinkFramePositionID);
  jdoubleArray jworldLinkFrameOrientation = (jdoubleArray)env->GetObjectField(jv, worldLinkFrameOrientationID);
  jdoubleArray jworldLinearVelocity = (jdoubleArray)env->GetObjectField(jv, worldLinearVelocityID);
  jdoubleArray jworldAngularVelocity = (jdoubleArray)env->GetObjectField(jv, worldAngularVelocityID);
  jdoubleArray jworldAABBMin = (jdoubleArray)env->GetObjectField(jv, worldAABBMinID);
  jdoubleArray jworldAABBMax = (jdoubleArray)env->GetObjectField(jv, worldAABBMaxID);

  CHECKVALUE(jworldPosition, "LinkState: worldPosition is null",);
  CHECKVALUE(jworldOrientation, "LinkState: worldOrientation is null",);
  CHECKVALUE(jlocalInertialPosition, "LinkState: localInertialPosition is null",); 
  CHECKVALUE(jlocalInertialOrientation, "LinkState: localInertialOrientation is null",);
  CHECKVALUE(jworldLinkFramePosition, "LinkState: worldLinkFramePosition is null",);
  CHECKVALUE(jworldLinkFrameOrientation, "LinkState: worldLinkFrameOrientation is null",);
  CHECKVALUE(jworldLinearVelocity, "LinkState: worldLinearVelocity is null",); 
  CHECKVALUE(jworldAngularVelocity, "LinkState: worldAngularVelocity is null",);
  CHECKVALUE(jworldAABBMin, "LinkState: worldAABBMin is null",);
  CHECKVALUE(jworldAABBMax, "LinkState: worldAABBMax is null",);

  double *worldPosition = env->GetDoubleArrayElements(jworldPosition, NULL);
  double *worldOrientation = env->GetDoubleArrayElements(jworldOrientation, NULL);
  double *localInertialPosition = env->GetDoubleArrayElements(jlocalInertialPosition, NULL);
  double *localInertialOrientation = env->GetDoubleArrayElements(jlocalInertialOrientation, NULL);
  double *worldLinkFramePosition = env->GetDoubleArrayElements(jworldLinkFramePosition, NULL);
  double *worldLinkFrameOrientation = env->GetDoubleArrayElements(jworldLinkFrameOrientation, NULL);
  double *worldLinearVelocity = env->GetDoubleArrayElements(jworldLinearVelocity, NULL);
  double *worldAngularVelocity = env->GetDoubleArrayElements(jworldAngularVelocity, NULL);
  double *worldAABBMin = env->GetDoubleArrayElements(jworldAABBMin, NULL);
  double *worldAABBMax = env->GetDoubleArrayElements(jworldAABBMax, NULL);

  worldPosition[0] = linkState.m_worldPosition[0];
  worldPosition[1] = linkState.m_worldPosition[1];
  worldPosition[2] = linkState.m_worldPosition[2];
  worldOrientation[0] = linkState.m_worldOrientation[0];
  worldOrientation[1] = linkState.m_worldOrientation[1];
  worldOrientation[2] = linkState.m_worldOrientation[2];
  worldOrientation[3] = linkState.m_worldOrientation[3];

  localInertialPosition[0] = linkState.m_localInertialPosition[0];
  localInertialPosition[1] = linkState.m_localInertialPosition[1];
  localInertialPosition[2] = linkState.m_localInertialPosition[2];
  localInertialOrientation[0] = linkState.m_localInertialOrientation[0];
  localInertialOrientation[1] = linkState.m_localInertialOrientation[1];
  localInertialOrientation[2] = linkState.m_localInertialOrientation[2];
  localInertialOrientation[3] = linkState.m_localInertialOrientation[3];

  worldLinkFramePosition[0] = linkState.m_worldLinkFramePosition[0];
  worldLinkFramePosition[1] = linkState.m_worldLinkFramePosition[1];
  worldLinkFramePosition[2] = linkState.m_worldLinkFramePosition[2];
  worldLinkFrameOrientation[0] = linkState.m_worldLinkFrameOrientation[0];
  worldLinkFrameOrientation[1] = linkState.m_worldLinkFrameOrientation[1];
  worldLinkFrameOrientation[2] = linkState.m_worldLinkFrameOrientation[2];
  worldLinkFrameOrientation[3] = linkState.m_worldLinkFrameOrientation[3];

  worldLinearVelocity[0] = linkState.m_worldLinearVelocity[0];
  worldLinearVelocity[1] = linkState.m_worldLinearVelocity[1];
  worldLinearVelocity[2] = linkState.m_worldLinearVelocity[2];
  worldAngularVelocity[0] = linkState.m_worldAngularVelocity[0];
  worldAngularVelocity[1] = linkState.m_worldAngularVelocity[1];
  worldAngularVelocity[2] = linkState.m_worldAngularVelocity[2];

  worldAABBMin[0] = linkState.m_worldAABBMin[0];
  worldAABBMin[1] = linkState.m_worldAABBMin[1];
  worldAABBMin[2] = linkState.m_worldAABBMin[2];
  worldAABBMax[0] = linkState.m_worldAABBMax[0];
  worldAABBMax[1] = linkState.m_worldAABBMax[1];
  worldAABBMax[2] = linkState.m_worldAABBMax[2];

  env->ReleaseDoubleArrayElements(jworldAABBMax, worldAABBMax, 0);
  env->ReleaseDoubleArrayElements(jworldAABBMin, worldAABBMin, 0);
  env->ReleaseDoubleArrayElements(jworldAngularVelocity, worldAngularVelocity, 0);
  env->ReleaseDoubleArrayElements(jworldLinearVelocity, worldLinearVelocity, 0);
  env->ReleaseDoubleArrayElements(jworldLinkFrameOrientation, worldLinkFrameOrientation, 0);
  env->ReleaseDoubleArrayElements(jworldLinkFramePosition, worldLinkFramePosition, 0);
  env->ReleaseDoubleArrayElements(jlocalInertialOrientation, localInertialOrientation, 0);
  env->ReleaseDoubleArrayElements(jlocalInertialPosition, localInertialPosition, 0);
  env->ReleaseDoubleArrayElements(jworldOrientation, worldOrientation, 0);
  env->ReleaseDoubleArrayElements(jworldPosition, worldPosition, 0);
}

static struct b3KeyboardEventsData javaKeyboardEventsDataToNative(JNIEnv *env, jobject jv) {
  int nevents, i;
  struct b3KeyboardEventsData data;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/KeyboardEventsData");
  CHECKFIELD(numKeyboardEventsID, env->GetFieldID(clazz, "m_numKeyboardEvents", "I"), "KeyboardEventsData: can't access m_numKeyboardEvents\n", data);
  CHECKFIELD(keyboardEventsID, env->GetFieldID(clazz, "m_keyboardEvents", "[Ledu/berkeley/bid/bullet/KeyboardEvent;"), "KeyboardEventsData: can't access m_keyboardEvents\n", data);
  jclass eclass = (jclass) env->FindClass("edu/berkeley/bid/bullet/KeyboardEvent");
  CHECKFIELD(keyCodeID, env->GetFieldID(eclass, "m_keyCode", "I"), "KeyboardEvent: can't access m_keyCode\n", data);
  CHECKFIELD(keyStateID, env->GetFieldID(eclass, "m_keyState", "I"), "KeyboardEvent: can't access m_keyState\n", data);
  nevents =  env->GetIntField(jv, numKeyboardEventsID);
  data.m_numKeyboardEvents = nevents;
  jobjectArray jkeyboardEvents = (jobjectArray)env->GetObjectField(jv, keyboardEventsID);
  if (nevents > 0) {
    CHECKVALUE(jkeyboardEvents, "KeyboardEventsData: m_keyboardEvents is null", data);
    data.m_keyboardEvents = new b3KeyboardEvent[nevents];
    for (i = 0; i < nevents; i++) {
      jobject keyEvent = env->GetObjectArrayElement(jkeyboardEvents, i);
      data.m_keyboardEvents[i].m_keyCode = env->GetIntField(keyEvent, keyCodeID);
      data.m_keyboardEvents[i].m_keyState = env->GetIntField(keyEvent, keyStateID);
    }
  }
  return data;
}

static void nativeKeyboardEventsDataToJava(JNIEnv *env, jobject jv, struct b3KeyboardEventsData &data) {
  int nevents, i;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/KeyboardEventsData");
  CHECKFIELD(numKeyboardEventsID, env->GetFieldID(clazz, "m_numKeyboardEvents", "I"), "KeyboardEventsData: can't access m_numKeyboardEvents\n",);
  CHECKFIELD(keyboardEventsID, env->GetFieldID(clazz, "m_keyboardEvents", "[Ledu/berkeley/bid/bullet/KeyboardEvent;"), "KeyboardEventsData: can't access m_keyboardEvents\n",);
  jclass eclass = (jclass) env->FindClass("edu/berkeley/bid/bullet/KeyboardEvent");
  jmethodID econstructor = env->GetMethodID(eclass, "<init>", "()V");
  if (econstructor == 0) {fprintf(stderr, "KeyboardEvent: can't access constructor\n"); return;}
  CHECKFIELD(keyCodeID, env->GetFieldID(eclass, "m_keyCode", "I"), "KeyboardEvent: can't access m_keyCode\n",);
  CHECKFIELD(keyStateID, env->GetFieldID(eclass, "m_keyState", "I"), "KeyboardEvent: can't access m_keyState\n",);
  nevents = data.m_numKeyboardEvents;
  env->SetIntField(jv, numKeyboardEventsID, nevents);
  if (nevents > 0) {
    jobjectArray jkeyboardEvents = env->NewObjectArray(nevents, eclass, NULL);
    env->SetObjectField(jv, keyboardEventsID, jkeyboardEvents);
    for (i = 0; i < nevents; i++) {
      jobject keyEvent = env->NewObject(eclass, econstructor);
      env->SetObjectArrayElement(jkeyboardEvents, i, keyEvent);
      env->SetIntField(keyEvent, keyCodeID, data.m_keyboardEvents[i].m_keyCode);
      env->SetIntField(keyEvent, keyStateID, data.m_keyboardEvents[i].m_keyState);
    }
  }
}

extern "C" {


JNIEXPORT jint JNICALL Java_edu_berkeley_bid_Bullet_newRobotSimulatorClientAPI
(JNIEnv *env, jobject jRoboSimAPI)
{
  b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();
  int status = (sim != NULL);
  
  setRobotSimulatorClientAPI(env, jRoboSimAPI, sim);
  
  return status;
}

JNIEXPORT jint JNICALL Java_edu_berkeley_bid_Bullet_deleteRobotSimulatorClientAPI
(JNIEnv *env, jobject jRoboSimAPI)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  delete [] jrsa;

  setRobotSimulatorClientAPI(env, jRoboSimAPI, NULL);

  return 0;
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_renderScene
(JNIEnv *env, jobject jRoboSimAPI)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> renderScene();
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_debugDraw
(JNIEnv *env, jobject jRoboSimAPI, jint debugDrawMode)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> debugDraw(debugDrawMode);
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_connect
(JNIEnv *env, jobject jRoboSimAPI, jint method, jstring jhostname, jint port)
{
  char *hostname = (char *)(env->GetStringUTFChars(jhostname, 0));
  
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  bool success = jrsa -> connect(method, hostname, port);
  
  env -> ReleaseStringUTFChars(jhostname, hostname);
  return success;
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_disconnect
(JNIEnv *env, jobject jRoboSimAPI)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> disconnect();
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_isConnected
(JNIEnv *env, jobject jRoboSimAPI)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  return jrsa -> isConnected();
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_setTimeOut
(JNIEnv *env, jobject jRoboSimAPI, jdouble t)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  return jrsa -> setTimeOut(t);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_syncBodies
(JNIEnv *env, jobject jRoboSimAPI)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> syncBodies();
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_resetSimulation
(JNIEnv *env, jobject jRoboSimAPI)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> resetSimulation();
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_getQuaternionFromEuler
(JNIEnv *env, jobject jRoboSimAPI, jobject jVec3, jobject jQuat)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  b3Vector3 bv = javaVector3ToNative(env, jVec3);
  b3Quaternion qv = jrsa -> getQuaternionFromEuler(bv);
  nativeQuaternionToJava(env, jQuat, qv);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_getEulerFromQuaternion
(JNIEnv *env, jobject jRoboSimAPI, jobject jQuat, jobject jVec3)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  b3Quaternion qv = javaQuaternionToNative(env, jQuat);
  b3Vector3 bv = jrsa -> getEulerFromQuaternion(qv);
  nativeVector3ToJava(env, jVec3, bv);
}

JNIEXPORT jint Java_edu_berkeley_bid_Bullet_loadURDF
(JNIEnv *env, jobject jRoboSimAPI, jstring jfname, jobject jStartPos, jobject jStartOrient, 
 jboolean jForceOverrideFixedBase, jboolean jUseMultiBody, jint jFlags)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  char *fname = (char *)(env->GetStringUTFChars(jfname, 0));
  b3Vector3 startPos = javaVector3ToNative(env, jStartPos);
  b3Quaternion startOrient = javaQuaternionToNative(env, jStartOrient);
  b3RobotSimulatorLoadUrdfFileArgs args(startPos, startOrient);
  args.m_forceOverrideFixedBase = jForceOverrideFixedBase;
  args.m_useMultiBody = jUseMultiBody;
  args.m_flags = jFlags;

  int results = jrsa -> loadURDF(fname, args);    

  env -> ReleaseStringUTFChars(jfname, fname);
  return results;
}

JNIEXPORT jintArray Java_edu_berkeley_bid_Bullet_loadSDF
(JNIEnv *env, jobject jRoboSimAPI, jstring jfname, jboolean jForceOverrideFixedBase, jboolean jUseMultiBody)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  char *fname = (char *)(env->GetStringUTFChars(jfname, 0));
  int i;

  b3RobotSimulatorLoadSdfFileArgs args;
  args.m_forceOverrideFixedBase = jForceOverrideFixedBase;
  args.m_useMultiBody = jUseMultiBody;

  b3RobotSimulatorLoadFileResults m_results;
  bool status = jrsa -> loadSDF(fname, m_results, args);    

  int size = m_results.m_uniqueObjectIds.size();
  jintArray results = env->NewIntArray(size);
  if (results != NULL) {
    jint *body = env->GetIntArrayElements(results, 0);
    for (i = 0; i < size; i++) {
      body[i] = m_results.m_uniqueObjectIds[i];
    }
    env->ReleaseIntArrayElements(results, body, 0);
  }
  env -> ReleaseStringUTFChars(jfname, fname);
  return results;
}

JNIEXPORT jintArray Java_edu_berkeley_bid_Bullet_loadMJCF
(JNIEnv *env, jobject jRoboSimAPI, jstring jfname)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  char *fname = (char *)(env->GetStringUTFChars(jfname, 0));
  int i;

  b3RobotSimulatorLoadFileResults m_results;
  bool status = jrsa -> loadMJCF(fname, m_results);    

  int size = m_results.m_uniqueObjectIds.size();
  jintArray results = env->NewIntArray(size);
  if (results != NULL) {
    jint *body = env->GetIntArrayElements(results, 0);
    for (i = 0; i < size; i++) {
      body[i] = m_results.m_uniqueObjectIds[i];
    }
    env->ReleaseIntArrayElements(results, body, 0);
  }
  env->ReleaseStringUTFChars(jfname, fname);
  return results;
}

JNIEXPORT jintArray Java_edu_berkeley_bid_Bullet_loadBullet
(JNIEnv *env, jobject jRoboSimAPI, jstring jfname)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  char *fname = (char *)(env->GetStringUTFChars(jfname, 0));
  int i;

  b3RobotSimulatorLoadFileResults m_results;
  bool status = jrsa -> loadBullet(fname, m_results);    

  int size = m_results.m_uniqueObjectIds.size();
  jintArray results = env->NewIntArray(size);
  if (results != NULL) {
    jint *body = env->GetIntArrayElements(results, 0);
    for (i = 0; i < size; i++) {
      body[i] = m_results.m_uniqueObjectIds[i];
    }
    env->ReleaseIntArrayElements(results, body, 0);
  }
  env->ReleaseStringUTFChars(jfname, fname);
  return results;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_getBodyInfo
(JNIEnv *env, jobject jRoboSimAPI, jint  bodyUniqueId, jobject jBodyInfo)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  struct b3BodyInfo bodyInfo;
  bool status = jrsa -> getBodyInfo(bodyUniqueId, &bodyInfo);
  jstring jbaseName = env->NewStringUTF(bodyInfo.m_baseName);
  jstring jbodyName = env->NewStringUTF(bodyInfo.m_bodyName);
  jclass clazz = env->GetObjectClass(jBodyInfo);
  CHECKFIELD(baseNameField, env->GetFieldID(clazz, "m_baseName", "Ljava/lang/String;"), "Bullet: can't access m_baseName field\n", false);
  CHECKFIELD(bodyNameField, env->GetFieldID(clazz, "m_bodyName", "Ljava/lang/String;"), "Bullet: can't access m_bodyName field\n", false);
  env->SetObjectField(jBodyInfo, baseNameField, jbaseName);
  env->SetObjectField(jBodyInfo, bodyNameField, jbodyName);
  return status;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_getBasePositionAndOrientation
(JNIEnv *env, jobject jRoboSimAPI, jint  bodyUniqueId, jobject jBasePosition, jobject jBaseOrient)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  b3Vector3 basePosition;
  b3Quaternion baseOrient;
  bool status = jrsa -> getBasePositionAndOrientation(bodyUniqueId, basePosition, baseOrient);
  nativeVector3ToJava(env, jBasePosition, basePosition);
  nativeQuaternionToJava(env, jBaseOrient, baseOrient);
  return status;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_resetBasePositionAndOrientation
(JNIEnv *env, jobject jRoboSimAPI, jint  bodyUniqueId, jobject jBasePosition, jobject jBaseOrient)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  b3Vector3 basePosition = javaVector3ToNative(env, jBasePosition);
  b3Quaternion baseOrient = javaQuaternionToNative(env, jBaseOrient);
  bool status = jrsa -> resetBasePositionAndOrientation(bodyUniqueId, basePosition, baseOrient);
  return status;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_getBaseVelocity
(JNIEnv *env, jobject jRoboSimAPI, jint  bodyUniqueId, jobject jBaseVelocity, jobject jBaseAngularV)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  b3Vector3 baseVelocity;
  b3Vector3 baseAngularV;
  bool status = jrsa -> getBaseVelocity(bodyUniqueId, baseVelocity, baseAngularV);
  nativeVector3ToJava(env, jBaseVelocity, baseVelocity);
  nativeVector3ToJava(env, jBaseAngularV, baseAngularV);
  return status;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_resetBaseVelocity
(JNIEnv *env, jobject jRoboSimAPI, jint  bodyUniqueId, jobject jBaseVelocity, jobject jBaseAngularV)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  b3Vector3 baseVelocity = javaVector3ToNative(env, jBaseVelocity);
  b3Vector3 baseAngularV = javaVector3ToNative(env, jBaseAngularV);
  bool status = jrsa -> resetBaseVelocity(bodyUniqueId, baseVelocity, baseAngularV);
  return status;
}

JNIEXPORT jint Java_edu_berkeley_bid_Bullet_getNumJoints
(JNIEnv *env, jobject jRoboSimAPI, jint bodyUniqueId)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  int njoints = jrsa -> getNumJoints(bodyUniqueId);
  return njoints;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_getJointInfo
(JNIEnv *env, jobject jRoboSimAPI, jint bodyUniqueId, jint jointIndex, jobject jJointInfo)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  struct b3JointInfo jointInfo;
  bool status = jrsa -> getJointInfo(bodyUniqueId, jointIndex, &jointInfo);
  nativeJointInfoToJava(env, jJointInfo, jointInfo);
  
  return status;
}

JNIEXPORT jint Java_edu_berkeley_bid_Bullet_createConstraint
(JNIEnv *env, jobject jRoboSimAPI, jint parentBodyIndex, jint parentJointIndex, jint childBodyIndex, jint childJointIndex, jobject jJointInfo)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  struct b3JointInfo jointInfo = javaJointInfoToNative(env, jJointInfo);
  int constraintId = jrsa -> createConstraint(parentBodyIndex, parentJointIndex, childBodyIndex, childJointIndex, &jointInfo);
  
  return constraintId;
}

JNIEXPORT jint Java_edu_berkeley_bid_Bullet_changeConstraint
(JNIEnv *env, jobject jRoboSimAPI, jint constraintId, jobject jJointInfo)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  struct b3JointInfo jointInfo = javaJointInfoToNative(env, jJointInfo);
  int retval = jrsa -> changeConstraint(constraintId, &jointInfo);
  
  return retval;
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_removeConstraint
(JNIEnv *env, jobject jRoboSimAPI, jint constraintId)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> removeConstraint(constraintId);
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_getJointState
(JNIEnv *env, jobject jRoboSimAPI, jint bodyUniqueId, jint jointIndex, jobject state)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  b3JointSensorState jointSensorState;
  bool status = jrsa -> getJointState(bodyUniqueId, jointIndex, &jointSensorState);
  nativeJointSensorStateToJava(env, state, jointSensorState);
  return status;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_getJointStates
(JNIEnv *env, jobject jRoboSimAPI, jint bodyUniqueId, jobject state)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  b3JointStates2 jointStates;
  bool status = jrsa -> getJointStates(bodyUniqueId, jointStates);
  int numJoints = jrsa -> getNumJoints(bodyUniqueId);
  nativeJointStates2ToJava(env, state, jointStates, numJoints);
  return status;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_resetJointState
(JNIEnv *env, jobject jRoboSimAPI, jint bodyUniqueId, jint jointIndex, jdouble targetValue)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  int status = jrsa -> resetJointState(bodyUniqueId, jointIndex, targetValue);
  return status;
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_setJointMotorControl
(JNIEnv *env, jobject jRoboSimAPI, jint bodyUniqueId, jint jointIndex, jobject args)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  struct b3RobotSimulatorJointMotorArgs motorArgs = javaJointMotorArgsToNative(env, args);
  jrsa -> setJointMotorControl(bodyUniqueId, jointIndex, motorArgs);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_stepSimulation
(JNIEnv *env, jobject jRoboSimAPI)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> stepSimulation();
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_canSubmitCommand
(JNIEnv *env, jobject jRoboSimAPI)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  return jrsa -> canSubmitCommand();
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_setRealTimeSimulation
(JNIEnv *env, jobject jRoboSimAPI, jboolean enableRealTimeSimulation)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> setRealTimeSimulation(enableRealTimeSimulation);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_setInternalSimFlags
(JNIEnv *env, jobject jRoboSimAPI, jint flags)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> setInternalSimFlags(flags);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_setGravity
(JNIEnv *env, jobject jRoboSimAPI, jobject jgravity)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  b3Vector3 gravity = javaVector3ToNative(env, jgravity);
  jrsa -> setGravity(gravity);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_setTimeStep
(JNIEnv *env, jobject jRoboSimAPI, jdouble t)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> setTimeStep(t);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_setNumSimulationSubSteps
(JNIEnv *env, jobject jRoboSimAPI, jint numSubSteps)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> setNumSimulationSubSteps(numSubSteps);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_setNumSolverIterations
(JNIEnv *env, jobject jRoboSimAPI, jint numSolverIterations)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> setNumSolverIterations(numSolverIterations);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_setContactBreakingThreshold
(JNIEnv *env, jobject jRoboSimAPI, jdouble threshold)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> setContactBreakingThreshold(threshold);
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_calculateInverseKinematics
(JNIEnv *env, jobject jRoboSimAPI, jobject jargs, jobject jresults)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  struct b3RobotSimulatorInverseKinematicArgs args = javaInverseKinematicArgsToNative(env, jargs);
  struct b3RobotSimulatorInverseKinematicsResults results;
  jboolean status = jrsa -> calculateInverseKinematics(args, results);
  nativeInverseKinematicsResultsToJava(env, jresults, results);
  return status;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_getBodyJacobian
(JNIEnv *env, jobject jRoboSimAPI, jint bodyUniqueId, jint linkIndex, jdoubleArray jlocalPosition, jdoubleArray jjointPositions,
 jdoubleArray jjointVelocities, jdoubleArray jjointAccelerations, jdoubleArray jlinearJacobian, jdoubleArray jangularJacobian)
{
  int n, i;
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  CHECKVALUE(jlocalPosition, "getBodyJacobian: localPosition is null", false);
  CHECKVALUE(jjointPositions, "getBodyJacobian: jointPositions is null", false);
  CHECKVALUE(jjointVelocities, "getBodyJacobian: jointVelocities is null", false);
  CHECKVALUE(jjointAccelerations, "getBodyJacobian: jointAccelerations is null", false);
  CHECKVALUE(jlinearJacobian, "getBodyJacobian: linearJacobian is null", false);
  CHECKVALUE(jangularJacobian, "getBodyJacobian: angularJacobian is null", false);

  double *localPosition = (jdouble *)env->GetPrimitiveArrayCritical(jlocalPosition, JNI_FALSE);
  double *jointPositions = (jdouble *)env->GetPrimitiveArrayCritical(jjointPositions, JNI_FALSE);
  double *jointVelocities = (jdouble *)env->GetPrimitiveArrayCritical(jjointVelocities, JNI_FALSE);
  double *jointAccelerations = (jdouble *)env->GetPrimitiveArrayCritical(jjointAccelerations, JNI_FALSE);
  double *linearJacobian = (jdouble *)env->GetPrimitiveArrayCritical(jlinearJacobian, JNI_FALSE);
  double *angularJacobian = (jdouble *)env->GetPrimitiveArrayCritical(jangularJacobian, JNI_FALSE);

  bool status = jrsa -> getBodyJacobian(bodyUniqueId, linkIndex, localPosition, jointPositions, jointVelocities, jointAccelerations, linearJacobian, angularJacobian);
  
  env->ReleasePrimitiveArrayCritical(jangularJacobian, angularJacobian, 0);
  env->ReleasePrimitiveArrayCritical(jlinearJacobian, linearJacobian, 0);
  env->ReleasePrimitiveArrayCritical(jjointAccelerations, jointAccelerations, 0);
  env->ReleasePrimitiveArrayCritical(jjointVelocities, jointVelocities, 0);
  env->ReleasePrimitiveArrayCritical(jjointPositions, jointPositions, 0);
  env->ReleasePrimitiveArrayCritical(jlocalPosition, localPosition, 0);
  return status;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_getLinkState
(JNIEnv *env, jobject jRoboSimAPI, jint bodyUniqueId, jint linkIndex, jobject jlinkState)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  b3LinkState linkState;
  bool status = jrsa -> getLinkState(bodyUniqueId, linkIndex, &linkState);
  nativeLinkStateToJava(env, jlinkState, linkState);
  return status; 
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_configureDebugVisualizer
(JNIEnv *env, jobject jRoboSimAPI, jint flags, jint enable)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> configureDebugVisualizer((b3ConfigureDebugVisualizerEnum)flags, enable);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_resetDebugVisualizerCamera
(JNIEnv *env, jobject jRoboSimAPI, jdouble cameraDistance, jdouble cameraPitch, jdouble cameraYaw, jobject jtargetPos)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  b3Vector3 targetPos = javaVector3ToNative(env, jtargetPos);
  jrsa -> resetDebugVisualizerCamera(cameraDistance, cameraPitch, cameraYaw, targetPos);
}

JNIEXPORT jint Java_edu_berkeley_bid_Bullet_startStateLogging
(JNIEnv *env, jobject jRoboSimAPI, jint loggingType, jstring jfileName, jintArray jobjectUniqueIds, jint maxLogDof)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  CHECKVALUE(jfileName, "startStateLogging: fileName is null", 0);
  CHECKVALUE(jobjectUniqueIds, "startStateLogging: objectUniqueIds array is null", 0);
  const char *fileName = env->GetStringUTFChars(jfileName, NULL);
  jint *arrayObjectUniqueIds = env->GetIntArrayElements(jobjectUniqueIds, NULL);
  jint size = env->GetArrayLength(jobjectUniqueIds);
  b3AlignedObjectArray<int> objectUniqueIds = b3AlignedObjectArray<int>();
  objectUniqueIds.resize(size);
  for (int i = 0; i < size; i++) {
    objectUniqueIds[i] = arrayObjectUniqueIds[i];
  }
  jint logid = jrsa -> startStateLogging((b3StateLoggingType)loggingType, fileName, objectUniqueIds, maxLogDof);
  env->ReleaseIntArrayElements(jobjectUniqueIds, arrayObjectUniqueIds, 0);
  env->ReleaseStringUTFChars(jfileName, fileName);
  return logid;
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_stopStateLogging
(JNIEnv *env, jobject jRoboSimAPI, jint stateLoggerUniqueId)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> stopStateLogging(stateLoggerUniqueId);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_getKeyboardEventsData
(JNIEnv *env, jobject jRoboSimAPI, jobject jkeyboardEventsData)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  struct b3KeyboardEventsData keyboardEventsData;
  jrsa -> getKeyboardEvents(&keyboardEventsData);
  nativeKeyboardEventsDataToJava(env, jkeyboardEventsData, keyboardEventsData);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_submitProfileTiming
(JNIEnv *env, jobject jRoboSimAPI, jstring jprofileName, int durationInMicroseconds)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  CHECKVALUE(jprofileName, "submitProfileTiming: profileName is null",);
  const char *profileName = env->GetStringUTFChars(jprofileName, NULL);

  jrsa -> submitProfileTiming(profileName, durationInMicroseconds);

  env->ReleaseStringUTFChars(jprofileName, profileName);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_testMatrix3x3
(JNIEnv *env, jobject obj, jobject min, jobject mout)
{
  b3Matrix3x3 m = javaMatrix3x3ToNative(env, min);
  nativeMatrix3x3ToJava(env, mout, m);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_testTransform3
(JNIEnv *env, jobject obj, jobject min, jobject mout)
{
  b3Transform m = javaTransform3ToNative(env, min);
  nativeTransform3ToJava(env, mout, m);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_testJointInfo
(JNIEnv *env, jobject obj, jobject min, jobject mout)
{
  b3JointInfo m = javaJointInfoToNative(env, min);
  nativeJointInfoToJava(env, mout, m);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_testJointSensorState
(JNIEnv *env, jobject obj, jobject min, jobject mout)
{
  b3JointSensorState m = javaJointSensorStateToNative(env, min);
  nativeJointSensorStateToJava(env, mout, m);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_testJointStates2
(JNIEnv *env, jobject obj, jobject min, jobject mout, jint numJoints)
{
  struct b3JointStates2 m = javaJointStates2ToNative(env, min, numJoints);
  nativeJointStates2ToJava(env, mout, m, numJoints);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_testJointMotorArgs
(JNIEnv *env, jobject obj, jobject min, jobject mout)
{
  struct b3RobotSimulatorJointMotorArgs m = javaJointMotorArgsToNative(env, min);
  nativeJointMotorArgsToJava(env, mout, m);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_testLinkState
(JNIEnv *env, jobject obj, jobject min, jobject mout)
{
  struct b3LinkState m = javaLinkStateToNative(env, min);
  nativeLinkStateToJava(env, mout, m);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_testKeyboardEventsData
(JNIEnv *env, jobject obj, jobject min, jobject mout)
{
  struct b3KeyboardEventsData m = javaKeyboardEventsDataToNative(env, min);
  nativeKeyboardEventsDataToJava(env, mout, m);
}

}

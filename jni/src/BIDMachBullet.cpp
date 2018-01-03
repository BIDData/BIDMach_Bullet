#include <jni.h>
#include <../examples/SharedMemory/PhysicsClientC_API.h>
#include <../examples/RobotSimulator/b3RobotSimulatorClientAPI.h>
#include <string.h>

#define CHECKFIELD(FIELDID,EXPR,MSG,RETV) \
  jfieldID FIELDID = (EXPR); \
  if (FIELDID == NULL) { \
    b3Warning(MSG); \
    return RETV; \
  }

#define CHECKVALUE(VALUEID,MSG,RETV) \
  if (VALUEID == NULL) { \
    b3Warning(MSG); \
    return RETV; \
  }

#define CHECKDIMS(JARRAY,NDIMS,MSG,RETV)	\
  if (env->GetArrayLength(JARRAY) != NDIMS) {	\
    b3Warning(MSG); \
    return RETV; \
  }

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

// TODO: wont need this after 2.8.7 since these fields change from char * to char[1024] in b3JointInfo

static void deleteJointInfo(b3JointInfo * ptr) {
  if (ptr != NULL) {
    if (ptr->m_linkName != NULL) {
      delete [] ptr->m_linkName;
    }
    if (ptr->m_jointName != NULL) {
      delete [] ptr->m_jointName;
    }
    delete ptr;
  }
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
  if (jq != NULL) {
    getQuaternionFields(env, jqclass, jxfield, jyfield, jzfield, jwfield);
    x = env->GetFloatField(jq, jxfield);
    y = env->GetFloatField(jq, jyfield);
    z = env->GetFloatField(jq, jzfield);
    w = env->GetFloatField(jq, jwfield);
    return b3Quaternion(x, y, z, w);
  } else {
    return b3Quaternion(0, 0, 0, 1);
  }
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
  if (jv != NULL) {
    getVector3Fields(env, jvclass, jxfield, jyfield, jzfield);
    x = env->GetFloatField(jv, jxfield);
    y = env->GetFloatField(jv, jyfield);
    z = env->GetFloatField(jv, jzfield);
    return b3MakeVector3(x, y, z);
  } else {
    return b3MakeVector3(0, 0, 0);
  }
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

static b3JointInfo * javaJointInfoToNative(JNIEnv *env, jobject jv) {
  b3JointInfo * jointInfo = new b3JointInfo();
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

  // TODO: will need to be changed for versions > 2.8.7 since these fields change from char * to char [1024]
  jointInfo->m_linkName = new char[strlen(linkName)+1];
  jointInfo->m_jointName = new char[strlen(jointName)+1];
  strcpy(jointInfo->m_linkName, linkName);
  strcpy(jointInfo->m_jointName, jointName);
  
  env -> ReleaseStringUTFChars(jJointName, jointName);
  env -> ReleaseStringUTFChars(jLinkName, linkName);

  jointInfo->m_jointType = env->GetIntField(jv, jointTypeID);
  jointInfo->m_qIndex = env->GetIntField(jv, qIndexID);
  jointInfo->m_uIndex = env->GetIntField(jv, uIndexID);
  jointInfo->m_jointIndex = env->GetIntField(jv, jointIndexID);
  jointInfo->m_flags = env->GetIntField(jv, flagsID);
  jointInfo->m_jointDamping = env->GetDoubleField(jv, jointDampingID);
  jointInfo->m_jointFriction = env->GetDoubleField(jv, jointFrictionID);
  jointInfo->m_jointLowerLimit = env->GetDoubleField(jv, jointLowerLimitID);
  jointInfo->m_jointUpperLimit = env->GetDoubleField(jv, jointUpperLimitID);
  jointInfo->m_jointMaxForce = env->GetDoubleField(jv, jointMaxForceID);
  jointInfo->m_jointMaxVelocity = env->GetDoubleField(jv, jointMaxVelocityID);

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
    jointInfo->m_parentFrame[i] = parentFrame[i];
  }
  for (i = 0; i < 7; i++) {
    jointInfo->m_childFrame[i] = childFrame[i];
  }
  for (i = 0; i < 3; i++) {
    jointInfo->m_jointAxis[i] = jointAxis[i];
  }

  env->ReleasePrimitiveArrayCritical(jJointAxis, jointAxis, 0);
  env->ReleasePrimitiveArrayCritical(jChildFrame, childFrame, 0);
  env->ReleasePrimitiveArrayCritical(jParentFrame, parentFrame, 0);

  return jointInfo;
}

static void nativeJointInfoToJava(JNIEnv *env, jobject jv, b3JointInfo *jointInfo) {
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

  jstring jlinkName = NULL;
  jstring jjointName = NULL;
  int len1 = strlen(jointInfo->m_linkName);
  int len2 = strlen(jointInfo->m_jointName);
  if (len1 > 0) jlinkName = env->NewStringUTF(jointInfo->m_linkName);
  if (len2 > 0) jjointName = env->NewStringUTF(jointInfo->m_jointName);
      
  env->SetObjectField(jv, linkNameID, jlinkName);
  env->SetObjectField(jv, jointNameID, jjointName);
  env->SetIntField(jv, jointTypeID, jointInfo->m_jointType);
  env->SetIntField(jv, qIndexID, jointInfo->m_qIndex);
  env->SetIntField(jv, uIndexID, jointInfo->m_uIndex);
  env->SetIntField(jv, jointIndexID, jointInfo->m_jointIndex);
  env->SetIntField(jv, flagsID, jointInfo->m_flags);
  env->SetDoubleField(jv, jointDampingID, jointInfo->m_jointDamping);
  env->SetDoubleField(jv, jointFrictionID, jointInfo->m_jointFriction);
  env->SetDoubleField(jv, jointLowerLimitID, jointInfo->m_jointLowerLimit);
  env->SetDoubleField(jv, jointUpperLimitID, jointInfo->m_jointUpperLimit);
  env->SetDoubleField(jv, jointMaxForceID, jointInfo->m_jointMaxForce);
  env->SetDoubleField(jv, jointMaxVelocityID, jointInfo->m_jointMaxVelocity);

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
    parentFrame[i] = jointInfo->m_parentFrame[i];
  }
  for (i = 0; i < 7; i++) {
    childFrame[i] = jointInfo->m_childFrame[i];
  }
  for (i = 0; i < 3; i++) {
    jointAxis[i] = jointInfo->m_jointAxis[i];
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

static struct b3DynamicsInfo javaDynamicsInfoToNative(JNIEnv *env, jobject jv) {
  int i;
  struct b3DynamicsInfo dynamicsInfo;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/DynamicsInfo");
  CHECKFIELD(massID, env->GetFieldID(clazz, "m_mass", "D"), "getDynamicsInfo: can't acccess m_mass\n", dynamicsInfo;);
  CHECKFIELD(lateralFrictionCoeffID, env->GetFieldID(clazz, "m_lateralFrictionCoeff", "D"), "getDynamicsInfo: can't acccess m_lateralFrictionCoeff\n", dynamicsInfo;);
  CHECKFIELD(localInertialPositionID, env->GetFieldID(clazz, "m_localInertialPosition", "[D"), "getDynamicsInfo: can't acccess m_localInertialPosition\n", dynamicsInfo;);

  dynamicsInfo.m_mass = env->GetDoubleField(jv, massID);
  dynamicsInfo.m_lateralFrictionCoeff = env->GetDoubleField(jv, lateralFrictionCoeffID);

  jdoubleArray jlocalInertialPosition = (jdoubleArray)env->GetObjectField(jv, localInertialPositionID);
  CHECKVALUE(jlocalInertialPosition, "getDynamicsInfo: m_localInertialPosition is null\n", dynamicsInfo);
  CHECKDIMS(jlocalInertialPosition, 3, "getDynamicsInfo: m_localInertialPosition dimension must be 3\n", dynamicsInfo);
  
  double *localInertialPosition = (jdouble *)env->GetPrimitiveArrayCritical(jlocalInertialPosition, JNI_FALSE);

  for (i = 0; i < 3; i++) {
    dynamicsInfo.m_localInertialPosition[i] = localInertialPosition[i];
  }

  env->ReleasePrimitiveArrayCritical(jlocalInertialPosition, localInertialPosition, 0);
  return dynamicsInfo;
}

static void nativeDynamicsInfoToJava(JNIEnv *env, jobject jv, b3DynamicsInfo &dynamicsInfo) {
  int i;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/DynamicsInfo");
  CHECKFIELD(massID, env->GetFieldID(clazz, "m_mass", "D"), "getDynamicsInfo: can't acccess m_mass\n",);
  CHECKFIELD(lateralFrictionCoeffID, env->GetFieldID(clazz, "m_lateralFrictionCoeff", "D"), "getDynamicsInfo: can't acccess m_lateralFrictionCoeff\n",);
  CHECKFIELD(localInertialPositionID, env->GetFieldID(clazz, "m_localInertialPosition", "[D"), "getDynamicsInfo: can't acccess m_localInertialPosition\n",);

  env->SetDoubleField(jv, massID, dynamicsInfo.m_mass);
  env->SetDoubleField(jv, lateralFrictionCoeffID, dynamicsInfo.m_lateralFrictionCoeff);

  jdoubleArray jlocalInertialPosition = (jdoubleArray)env->GetObjectField(jv, localInertialPositionID);
  CHECKVALUE(jlocalInertialPosition, "getDynamicsInfo: m_localInertialPosition is null\n",);
  CHECKDIMS(jlocalInertialPosition, 3, "getDynamicsInfo: m_localInertialPosition dimension must be 3\n",);
  
  double *localInertialPosition = (jdouble *)env->GetPrimitiveArrayCritical(jlocalInertialPosition, JNI_FALSE);

  for (i = 0; i < 3; i++) {
    localInertialPosition[i] = dynamicsInfo.m_localInertialPosition[i];
  }

  env->ReleasePrimitiveArrayCritical(jlocalInertialPosition, localInertialPosition, 0);
}

static b3JointStates2 javaJointStates2ToNative(JNIEnv *env, jobject jv, int numJoints) {
  int i;
  b3JointStates2 jointStates2;
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

static void nativeJointStates2ToJava(JNIEnv *env, jobject jv, b3JointStates2 &jointStates2, int numJoints) {
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

static jboolean nativeInverseKinematicsResultsToJava(JNIEnv *env, jdoubleArray jv, struct b3RobotSimulatorInverseKinematicsResults &results) {
  int i, n;
  n = results.m_calculatedJointPositions.size();
  CHECKVALUE(jv, "calculateInverseKinematics: output array is null", false);
  CHECKDIMS(jv, n, "calculateInverseKinematics: output array dims must be numObjects", false);
  jdouble *calculatedJointPositions = (jdouble *)env->GetPrimitiveArrayCritical(jv, JNI_FALSE);
  for (i = 0; i < n; i++) {
    calculatedJointPositions[i] = results.m_calculatedJointPositions[i];
  }
  env->ReleasePrimitiveArrayCritical(jv, calculatedJointPositions, 0);
  return true;
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

  CHECKDIMS(jworldPosition, 3, "LinkState: worldPosition dimension must be 3", linkState);
  CHECKDIMS(jworldOrientation, 4, "LinkState: worldOrientation dimension must be 4", linkState);
  CHECKDIMS(jlocalInertialPosition, 3, "LinkState: localInertialPosition dimension must be 3", linkState); 
  CHECKDIMS(jlocalInertialOrientation, 4, "LinkState: localInertialOrientation dimension must be 4", linkState);
  CHECKDIMS(jworldLinkFramePosition, 3, "LinkState: worldLinkFramePosition dimension must be 3", linkState);
  CHECKDIMS(jworldLinkFrameOrientation, 4, "LinkState: worldLinkFrameOrientation dimension must be 4", linkState);
  CHECKDIMS(jworldLinearVelocity, 3, "LinkState: worldLinearVelocity dimension must be 3", linkState); 
  CHECKDIMS(jworldAngularVelocity, 3, "LinkState: worldAngularVelocity dimension must be 3", linkState);
  CHECKDIMS(jworldAABBMin, 3, "LinkState: worldAABBMin dimension must be 3", linkState);
  CHECKDIMS(jworldAABBMax, 3, "LinkState: worldAABBMax dimension must be 3", linkState);

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

  CHECKDIMS(jworldPosition, 3, "LinkState: worldPosition dimension must be 3",);
  CHECKDIMS(jworldOrientation, 4, "LinkState: worldOrientation dimension must be 4",);
  CHECKDIMS(jlocalInertialPosition, 3, "LinkState: localInertialPosition dimension must be 3",); 
  CHECKDIMS(jlocalInertialOrientation, 4, "LinkState: localInertialOrientation dimension must be 4",);
  CHECKDIMS(jworldLinkFramePosition, 3, "LinkState: worldLinkFramePosition dimension must be 3",);
  CHECKDIMS(jworldLinkFrameOrientation, 4, "LinkState: worldLinkFrameOrientation dimension must be 4",);
  CHECKDIMS(jworldLinearVelocity, 3, "LinkState: worldLinearVelocity dimension must be 3",); 
  CHECKDIMS(jworldAngularVelocity, 3, "LinkState: worldAngularVelocity dimension must be 3",);
  CHECKDIMS(jworldAABBMin, 3, "LinkState: worldAABBMin dimension must be 3",);
  CHECKDIMS(jworldAABBMax, 3, "LinkState: worldAABBMax dimension must be 3",);

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

static struct b3CameraImageData javaCameraImageDataToNative(JNIEnv *env, jobject jv) {
  struct b3CameraImageData data;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/CameraImageData");

  CHECKFIELD(pixelWidthID, env->GetFieldID(clazz, "m_pixelWidth", "I"), "CameraImageData: can't access m_pixelWidth\n", data);
  CHECKFIELD(pixelHeightID, env->GetFieldID(clazz, "m_pixelHeight", "I"), "CameraImageData: can't access m_pixelHeight\n", data);
  CHECKFIELD(rgbColorDataID, env->GetFieldID(clazz, "m_rgbColorData", "[B"), "CameraImageData: can't access m_rgbColorData\n", data);
  CHECKFIELD(depthValuesID, env->GetFieldID(clazz, "m_depthValues", "[F"), "CameraImageData: can't access m_depthValues\n", data);
  CHECKFIELD(segmentationMaskValuesID, env->GetFieldID(clazz, "m_segmentationMaskValues", "[I"), "CameraImageData: can't access m_segmentationMaskValues\n", data);

  jint width = env->GetIntField(jv, pixelWidthID);
  jint height = env->GetIntField(jv, pixelHeightID);
  jbyteArray jrgbData = (jbyteArray)env->GetObjectField(jv, rgbColorDataID);
  jfloatArray jdepthValues = (jfloatArray)env->GetObjectField(jv, depthValuesID);
  jintArray jsegmentation = (jintArray)env->GetObjectField(jv, segmentationMaskValuesID);

  CHECKVALUE(jrgbData, "CameraImageData: m_rgbColorData is null", data);
  CHECKVALUE(jdepthValues, "CameraImageData: m_depthValues is null", data);
  CHECKVALUE(jsegmentation, "CameraImageData: m_segmentationMaskValues is null", data);

  data.m_pixelWidth = width;
  data.m_pixelHeight = height;
  int length = width * height;
  if (length > 0) {
    unsigned char *rgbData = (unsigned char *)env->GetPrimitiveArrayCritical(jrgbData, JNI_FALSE);
    float *depthValues = (float *)env->GetPrimitiveArrayCritical(jdepthValues, JNI_FALSE);
    int *segmentation = (int *)env->GetPrimitiveArrayCritical(jsegmentation, JNI_FALSE);

    unsigned char *newRgbData = new unsigned char[length*4];
    float *newDepthValues = new float[length];
    int *newSegmentation = new int[length];

    memcpy(newRgbData, rgbData, 4*length);
    memcpy(newDepthValues, depthValues, length*sizeof(float));
    memcpy(newSegmentation, segmentation, length*sizeof(int));

    data.m_rgbColorData = newRgbData;
    data.m_depthValues = newDepthValues;
    data.m_segmentationMaskValues = newSegmentation;

    env->ReleasePrimitiveArrayCritical(jsegmentation, segmentation, 0);
    env->ReleasePrimitiveArrayCritical(jdepthValues, depthValues, 0);
    env->ReleasePrimitiveArrayCritical(jrgbData, rgbData, 0);
  }
  return data;
}

static void nativeCameraImageDataToJava(JNIEnv *env, jobject jv, b3CameraImageData &data) {
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/CameraImageData");

  CHECKFIELD(pixelWidthID, env->GetFieldID(clazz, "m_pixelWidth", "I"), "CameraImageData: can't access m_pixelWidth\n",);
  CHECKFIELD(pixelHeightID, env->GetFieldID(clazz, "m_pixelHeight", "I"), "CameraImageData: can't access m_pixelHeight\n",);
  CHECKFIELD(rgbColorDataID, env->GetFieldID(clazz, "m_rgbColorData", "[B"), "CameraImageData: can't access m_rgbColorData\n",);
  CHECKFIELD(depthValuesID, env->GetFieldID(clazz, "m_depthValues", "[F"), "CameraImageData: can't access m_depthValues\n",);
  CHECKFIELD(segmentationMaskValuesID, env->GetFieldID(clazz, "m_segmentationMaskValues", "[I"), "CameraImageData: can't access m_segmentationMaskValues\n",);

  jint width = data.m_pixelWidth;
  jint height = data.m_pixelHeight;
  jint length = width * height;
  env->SetIntField(jv, pixelWidthID, width);
  env->SetIntField(jv, pixelHeightID, height);

  if (length > 0) {
    jbyteArray jrgbData = (jbyteArray)env->GetObjectField(jv, rgbColorDataID);
    jfloatArray jdepthValues = (jfloatArray)env->GetObjectField(jv, depthValuesID);
    jintArray jsegmentation = (jintArray)env->GetObjectField(jv, segmentationMaskValuesID);

    if (jrgbData == NULL || env->GetArrayLength(jrgbData) != 4*length) {
      jrgbData = env->NewByteArray(length*4);
      env->SetObjectField(jv, rgbColorDataID, jrgbData);
    }
    if (jdepthValues == NULL || env->GetArrayLength(jdepthValues) != length) {
      jdepthValues = env->NewFloatArray(length);
      env->SetObjectField(jv, depthValuesID, jdepthValues);
    }
    if (jsegmentation == NULL  || env->GetArrayLength(jsegmentation) != length) {
      jsegmentation = env->NewIntArray(length);
      env->SetObjectField(jv, segmentationMaskValuesID, jsegmentation);
    }

    unsigned char *rgbData = (unsigned char *)env->GetPrimitiveArrayCritical(jrgbData, JNI_FALSE);
    float *depthValues = (float *)env->GetPrimitiveArrayCritical(jdepthValues, JNI_FALSE);
    int *segmentation = (int *)env->GetPrimitiveArrayCritical(jsegmentation, JNI_FALSE);

    memcpy(rgbData, data.m_rgbColorData, 4*length);
    memcpy(depthValues, data.m_depthValues, length*sizeof(float));
    memcpy(segmentation, data.m_segmentationMaskValues, length*sizeof(int));

    env->ReleasePrimitiveArrayCritical(jsegmentation, segmentation, 0);
    env->ReleasePrimitiveArrayCritical(jdepthValues, depthValues, 0);
    env->ReleasePrimitiveArrayCritical(jrgbData, rgbData, 0);
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
  delete jrsa;

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

JNIEXPORT jint Java_edu_berkeley_bid_Bullet_getNumBodies
(JNIEnv *env, jobject jRoboSimAPI)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  int nbodies = jrsa -> getNumBodies();
  return nbodies;
}

JNIEXPORT jint Java_edu_berkeley_bid_Bullet_getBodyUniqueId
(JNIEnv *env, jobject jRoboSimAPI, jint bodyId)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  int uid = jrsa -> getBodyUniqueId(bodyId);
  return uid;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_removeBody
(JNIEnv *env, jobject jRoboSimAPI, jint bodyUniqueId)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  bool status = jrsa -> removeBody(bodyUniqueId);
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
  b3JointInfo *jointInfo = new b3JointInfo;
  bool status = jrsa -> getJointInfo(bodyUniqueId, jointIndex, jointInfo);
  nativeJointInfoToJava(env, jJointInfo, jointInfo);
  delete jointInfo;
  return status;
}

JNIEXPORT jint Java_edu_berkeley_bid_Bullet_createConstraint
(JNIEnv *env, jobject jRoboSimAPI, jint parentBodyIndex, jint parentJointIndex, jint childBodyIndex, jint childJointIndex, jobject jJointInfo)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  b3JointInfo *jointInfo = javaJointInfoToNative(env, jJointInfo);
  int constraintId = jrsa -> createConstraint(parentBodyIndex, parentJointIndex, childBodyIndex, childJointIndex, jointInfo);
  deleteJointInfo(jointInfo);
  return constraintId;
}

JNIEXPORT jint Java_edu_berkeley_bid_Bullet_changeConstraint
(JNIEnv *env, jobject jRoboSimAPI, jint constraintId, jobject jJointInfo)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  b3JointInfo *jointInfo = javaJointInfoToNative(env, jJointInfo);
  int retval = jrsa -> changeConstraint(constraintId, jointInfo);
  deleteJointInfo(jointInfo);
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
(JNIEnv *env, jobject jRoboSimAPI, jint bodyUniqueId, jint jointIndex,
 int controlMode,  double targetPosition, double targetVelocity, double force,
 double kp, double kd)
{
  struct b3RobotSimulatorJointMotorArgs motorArgs(controlMode);
  motorArgs.m_targetPosition = targetPosition;
  motorArgs.m_targetVelocity = targetVelocity;
  motorArgs.m_kp = kp;
  motorArgs.m_kd = kd;
  motorArgs.m_maxTorqueValue = force;

  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> setJointMotorControl(bodyUniqueId, jointIndex, motorArgs);
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_setJointMotorControlArray
(JNIEnv *env, jobject jRoboSimAPI, jint bodyUniqueId,  jintArray jjointIndices, int controlMode,
 jdoubleArray jtargetPositions, jdoubleArray jtargetVelocities, jdoubleArray jforces, jdoubleArray jkps, jdoubleArray jkds)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);

  int *jointIndices = NULL;
  double *targetPositions = NULL;
  double *targetVelocities = NULL;
  double *forces = NULL;
  double *kps = NULL;
  double *kds = NULL;

  CHECKVALUE(jjointIndices, "setJointMotorControlArray: jointIndices array is null", false);
  int numControlledDofs = env->GetArrayLength(jjointIndices);
  jointIndices = (int *)env->GetPrimitiveArrayCritical(jjointIndices, JNI_FALSE);

  if (jtargetPositions != NULL) {
    CHECKDIMS(jtargetPositions, numControlledDofs, "setJointMotorControlArray: targetPositions array dimension doesnt match number of jointIndices", false);
    targetPositions = (double *)env->GetPrimitiveArrayCritical(jtargetPositions, JNI_FALSE);
  }

  if (jtargetVelocities != NULL) {
    CHECKDIMS(jtargetVelocities, numControlledDofs, "setJointMotorControlArray: targetVelocities array dimension doesnt match number of jointIndices", false);
    targetVelocities = (double *)env->GetPrimitiveArrayCritical(jtargetVelocities, JNI_FALSE);
  }

  if (jforces != NULL) {
    CHECKDIMS(jforces, numControlledDofs, "setJointMotorControlArray: forces array dimension doesnt match number of jointIndices", false);
    forces = (double *)env->GetPrimitiveArrayCritical(jforces, JNI_FALSE);
  }

  if (jkps != NULL) {
    CHECKDIMS(jkps, numControlledDofs, "setJointMotorControlArray: kps array dimension doesnt match number of jointIndices", false);
    kps = (double *)env->GetPrimitiveArrayCritical(jkps, JNI_FALSE);
  }

  if (jkds != NULL) {
    CHECKDIMS(jkds, numControlledDofs, "setJointMotorControlArray: kds array dimension doesnt match number of jointIndices", false);
    kds = (double *)env->GetPrimitiveArrayCritical(jkds, JNI_FALSE);
  }

  struct b3RobotSimulatorJointMotorArrayArgs motorArgs(controlMode, numControlledDofs);
  motorArgs.m_jointIndices = jointIndices;
  motorArgs.m_targetPositions = targetPositions;
  motorArgs.m_targetVelocities = targetVelocities;
  motorArgs.m_forces = forces;
  motorArgs.m_kps = kps;
  motorArgs.m_kds = kds;

  bool status = jrsa -> setJointMotorControlArray(bodyUniqueId, motorArgs);

  if (kds != NULL) {
    env->ReleasePrimitiveArrayCritical(jkds, kds, 0);
  }

  if (kps != NULL) {
    env->ReleasePrimitiveArrayCritical(jkps, kps, 0);
  }

  if (forces != NULL) {
    env->ReleasePrimitiveArrayCritical(jforces, forces, 0);
  }

  if (targetVelocities != NULL) {
    env->ReleasePrimitiveArrayCritical(jtargetVelocities, targetVelocities, 0);
  }

  if (targetPositions != NULL) {
    env->ReleasePrimitiveArrayCritical(jtargetPositions, targetPositions, 0);
  }
  return status;
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
(JNIEnv *env, jobject jRoboSimAPI, jint bodyUniqueId, jint endEffectorLinkIndex,
 jdoubleArray jendEffectorTargetPosition, jdoubleArray jendEffectorTargetOrientation, 
 jdoubleArray jlowerLimits, jdoubleArray jupperLimits, jdoubleArray jjointRanges, jdoubleArray jrestPoses,
 jdoubleArray jjointDamping, jdoubleArray jresults)
{
  int i;
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  struct b3RobotSimulatorInverseKinematicArgs args;
  jdouble *endEffectorTargetPosition = NULL;
  jdouble *endEffectorTargetOrientation = NULL;
  jdouble *lowerLimits = NULL;
  jdouble *upperLimits = NULL;
  jdouble *jointRanges = NULL;
  jdouble *restPoses = NULL;
  jdouble *jointDamping = NULL;
  int numJoints = jrsa->getNumJoints(bodyUniqueId);

  args.m_flags = 0;
  args.m_bodyUniqueId = bodyUniqueId;
  args.m_endEffectorLinkIndex = endEffectorLinkIndex;

  // Process required args endEffectorTargetPosition
  CHECKVALUE(jendEffectorTargetPosition, "calculateInverseKinematics: m_endEffectorTargetPosition is null\n", false);
  CHECKDIMS(jendEffectorTargetPosition, 3, "calculateInverseKinematics: endEffectorTargetPosition dimension must be 3", false);
  endEffectorTargetPosition = (jdouble *)env->GetPrimitiveArrayCritical(jendEffectorTargetPosition, JNI_FALSE);
  args.m_endEffectorTargetPosition[0] = endEffectorTargetPosition[0];
  args.m_endEffectorTargetPosition[1] = endEffectorTargetPosition[1];
  args.m_endEffectorTargetPosition[2] = endEffectorTargetPosition[2];
  env->ReleasePrimitiveArrayCritical(jendEffectorTargetPosition, endEffectorTargetPosition, 0);
  
  // Process optional arg endEffectorTargetOrientation
  if (jendEffectorTargetOrientation != NULL) {
    CHECKDIMS(jendEffectorTargetOrientation, 4, "calculateInverseKinematics: endEffectorTargetOrientation dimension must be 4", false);
    endEffectorTargetOrientation = (jdouble *)env->GetPrimitiveArrayCritical(jendEffectorTargetOrientation, JNI_FALSE);
    args.m_endEffectorTargetOrientation[0] = endEffectorTargetOrientation[0];
    args.m_endEffectorTargetOrientation[1] = endEffectorTargetOrientation[1];
    args.m_endEffectorTargetOrientation[2] = endEffectorTargetOrientation[2];
    args.m_endEffectorTargetOrientation[3] = endEffectorTargetOrientation[3];
    env->ReleasePrimitiveArrayCritical(jendEffectorTargetOrientation, endEffectorTargetOrientation, 0);
    args.m_flags |= B3_HAS_IK_TARGET_ORIENTATION;
  }

  // Process optional arg group upperLimits, lowerLimits, jointRanges and restPoses
  if (jlowerLimits != NULL) {
    CHECKVALUE(jupperLimits, "calculateInverseKinematics: m_upperLimits is null\n", false);
    CHECKVALUE(jjointRanges, "calculateInverseKinematics: m_jointRanges is null\n", false);
    CHECKVALUE(jrestPoses, "calculateInverseKinematics: m_restPoses is null\n", false);
    CHECKDIMS(jlowerLimits, numJoints, "calculateInverseKinematics: lowerLimits dimension must be numJoints", false);
    CHECKDIMS(jupperLimits, numJoints, "calculateInverseKinematics: upperLimits dimension must be numJoints", false);
    CHECKDIMS(jjointRanges, numJoints, "calculateInverseKinematics: jointRanges dimension must be numJoints", false);
    CHECKDIMS(jrestPoses, numJoints, "calculateInverseKinematics: restPoses dimension must be numJoints", false);
    lowerLimits = (jdouble *)env->GetPrimitiveArrayCritical(jlowerLimits, JNI_FALSE);
    upperLimits = (jdouble *)env->GetPrimitiveArrayCritical(jupperLimits, JNI_FALSE);
    jointRanges = (jdouble *)env->GetPrimitiveArrayCritical(jjointRanges, JNI_FALSE);
    restPoses = (jdouble *)env->GetPrimitiveArrayCritical(jrestPoses, JNI_FALSE);
    args.m_lowerLimits.resize(numJoints);
    for (i = 0; i < numJoints; i++) {
      args.m_lowerLimits[i] = lowerLimits[i];
    }
    args.m_upperLimits.resize(numJoints);
    for (i = 0; i < numJoints; i++) {
      args.m_upperLimits[i] = upperLimits[i];
    }
    args.m_jointRanges.resize(numJoints);
    for (i = 0; i < numJoints; i++) {
      args.m_jointRanges[i] = jointRanges[i];
    }
    args.m_restPoses.resize(numJoints);
    for (i = 0; i < numJoints; i++) {
      args.m_restPoses[i] = restPoses[i];
    }
    env->ReleasePrimitiveArrayCritical(jrestPoses, restPoses, 0);
    env->ReleasePrimitiveArrayCritical(jjointRanges, jointRanges, 0);
    env->ReleasePrimitiveArrayCritical(jupperLimits, upperLimits, 0);
    env->ReleasePrimitiveArrayCritical(jlowerLimits, lowerLimits, 0);
    args.m_flags |= B3_HAS_NULL_SPACE_VELOCITY;
  }

  if (jjointDamping != NULL) {
    CHECKDIMS(jjointDamping, numJoints, "calculateInverseKinematics: jointDamping dimension must be numJoints", false);
    jointDamping = (jdouble *)env->GetPrimitiveArrayCritical(jjointDamping, JNI_FALSE);
    args.m_jointDamping.resize(numJoints);
    for (i = 0; i < numJoints; i++) {
      args.m_jointDamping[i] = jointDamping[i];
    }
    env->ReleasePrimitiveArrayCritical(jjointDamping, jointDamping, 0);
    args.m_flags |= B3_HAS_JOINT_DAMPING;
  }

  struct b3RobotSimulatorInverseKinematicsResults results;
  jboolean status = jrsa -> calculateInverseKinematics(args, results);
  if (status) status = nativeInverseKinematicsResultsToJava(env, jresults, results);
  return status;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_calculateInverseDynamics
(JNIEnv *env, jobject jRoboSimAPI, int bodyUniqueId, jdoubleArray jjointPositions, jdoubleArray jjointVelocities,
 jdoubleArray jjointAccelerations, jdoubleArray jjointForcesOutput) 
{
  int n, i;
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  int numJoints = jrsa->getNumJoints(bodyUniqueId);

  CHECKVALUE(jjointPositions, "calculateInverseDynamics: jointPositions is null", false);
  CHECKVALUE(jjointVelocities, "calculateInverseDynamics: jointVelocities is null", false);
  CHECKVALUE(jjointAccelerations, "calculateInverseDynamics: jointAccelerations is null", false);
  CHECKVALUE(jjointForcesOutput, "calculateInverseDynamics: jointForcesOutput is null", false);

  CHECKDIMS(jjointPositions, numJoints, "calculateInverseKinematics: jointPositions dimension must be numJoints", false);
  CHECKDIMS(jjointVelocities, numJoints, "calculateInverseKinematics: jointVelocities dimension must be numJoints", false);
  CHECKDIMS(jjointAccelerations, numJoints, "calculateInverseKinematics: jointAccelerations dimension must be numJoints", false);
  CHECKDIMS(jjointForcesOutput, numJoints, "calculateInverseKinematics: jointForcesOutput dimension must be numJoints", false);

  double *jointPositions = (jdouble *)env->GetPrimitiveArrayCritical(jjointPositions, JNI_FALSE);
  double *jointVelocities = (jdouble *)env->GetPrimitiveArrayCritical(jjointVelocities, JNI_FALSE);
  double *jointAccelerations = (jdouble *)env->GetPrimitiveArrayCritical(jjointAccelerations, JNI_FALSE);
  double *jointForcesOutput = (jdouble *)env->GetPrimitiveArrayCritical(jjointForcesOutput, JNI_FALSE);

  bool status = jrsa -> calculateInverseDynamics(bodyUniqueId, jointPositions, jointVelocities, jointAccelerations, jointForcesOutput);

  env->ReleasePrimitiveArrayCritical(jjointForcesOutput, jointForcesOutput, 0);
  env->ReleasePrimitiveArrayCritical(jjointAccelerations, jointAccelerations, 0);
  env->ReleasePrimitiveArrayCritical(jjointVelocities, jointVelocities, 0);
  env->ReleasePrimitiveArrayCritical(jjointPositions, jointPositions, 0);

  return status;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_getBodyJacobian
(JNIEnv *env, jobject jRoboSimAPI, jint bodyUniqueId, jint linkIndex, jdoubleArray jlocalPosition, jdoubleArray jjointPositions,
 jdoubleArray jjointVelocities, jdoubleArray jjointAccelerations, jdoubleArray jlinearJacobian, jdoubleArray jangularJacobian)
{
  int n, i;
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  int numJoints = jrsa->getNumJoints(bodyUniqueId);
  
  CHECKVALUE(jlocalPosition, "getBodyJacobian: localPosition is null", false);
  CHECKVALUE(jjointPositions, "getBodyJacobian: jointPositions is null", false);
  CHECKVALUE(jjointVelocities, "getBodyJacobian: jointVelocities is null", false);
  CHECKVALUE(jjointAccelerations, "getBodyJacobian: jointAccelerations is null", false);
  CHECKVALUE(jlinearJacobian, "getBodyJacobian: linearJacobian is null", false);
  CHECKVALUE(jangularJacobian, "getBodyJacobian: angularJacobian is null", false);

  CHECKDIMS(jlocalPosition, numJoints, "getBodyJacobian: localPosition dimension must be numJoints", false);
  CHECKDIMS(jjointPositions, numJoints, "getBodyJacobian: jointPositions dimension must be numJoints", false);
  CHECKDIMS(jjointVelocities, numJoints, "getBodyJacobian: jointVelocities dimension must be numJoints", false);
  CHECKDIMS(jjointAccelerations, numJoints, "getBodyJacobian: jointAccelerations dimension must be numJoints", false);
  CHECKDIMS(jlinearJacobian, 3*numJoints, "getBodyJacobian: linearJacobian dimension must be 3 x numJoints", false);
  CHECKDIMS(jangularJacobian, 3*numJoints, "getBodyJacobian: angularJacobian dimension must be 3 x numJoints", false);

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

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_getDynamicsInfo
(JNIEnv *env, jobject jRoboSimAPI, jint bodyUniqueId, jint jointIndex, jobject jDynamicsInfo)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  b3DynamicsInfo dynamicsInfo;
  bool status = jrsa -> getDynamicsInfo(bodyUniqueId, jointIndex, &dynamicsInfo);
  nativeDynamicsInfoToJava(env, jDynamicsInfo, dynamicsInfo);
  return status;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_changeDynamics
(JNIEnv *env, jobject jRoboSimAPI,
 jint bodyUniqueId, jint linkIndex, jdouble mass, jdouble lateralFriction, jdouble spinningFriction,
 jdouble rollingFriction, jdouble restitution, jdouble linearDamping, jdouble angularDamping,
 jdouble contactStiffness, jdouble contactDamping, jint frictionAnchor)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);

  bool status = jrsa -> changeDynamics(bodyUniqueId, linkIndex, mass, lateralFriction, spinningFriction,
				       rollingFriction, restitution, linearDamping, angularDamping,
				       contactStiffness, contactDamping, frictionAnchor);

  return status;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_getLinkState
(JNIEnv *env, jobject jRoboSimAPI, jint bodyUniqueId, jint linkIndex, int computeLinkVelocity, int computeForwardKinematics, jobject jlinkState)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  b3LinkState linkState;
  bool status = jrsa -> getLinkState(bodyUniqueId, linkIndex, computeLinkVelocity, computeForwardKinematics, &linkState);
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

JNIEXPORT void Java_edu_berkeley_bid_Bullet_computeViewMatrixFromPositions
(JNIEnv *env, jobject obj, jfloatArray jcameraPosition, jfloatArray jcameraTargetPosition,
 jfloatArray jcameraUp, jfloatArray jviewMatrix)
{
  float *cameraPosition = NULL;
  float *cameraTargetPosition = NULL;
  float *cameraUp = NULL;
  float *viewMatrix = NULL;

  CHECKVALUE(jcameraPosition, "computeViewMatrixFromPositions: cameraPosition is null",);
  CHECKDIMS(jcameraPosition, 3, "computeViewMatrixFromPositions: cameraPosition must have dimension 3",);
  CHECKVALUE(jcameraTargetPosition, "computeViewMatrixFromPositions: cameraTargetPosition is null",);
  CHECKDIMS(jcameraTargetPosition, 3, "computeViewMatrixFromPositions: cameraTargetPosition must have dimension 3",);
  CHECKVALUE(jcameraUp, "computeViewMatrixFromPositions: cameraUp is null",);
  CHECKDIMS(jcameraUp, 3, "computeViewMatrixFromPositions: cameraUp must have dimension 3",);
  CHECKVALUE(jviewMatrix, "computeViewMatrixFromPositions: viewMatrix is null",);
  CHECKDIMS(jviewMatrix, 16, "computeViewMatrixFromPositions: viewMatrix must have dimension 16",);

  cameraPosition = (float *)env->GetPrimitiveArrayCritical(jcameraPosition, JNI_FALSE);
  cameraTargetPosition = (float *)env->GetPrimitiveArrayCritical(jcameraTargetPosition, JNI_FALSE);
  cameraUp = (float *)env->GetPrimitiveArrayCritical(jcameraUp, JNI_FALSE);
  viewMatrix = (float *)env->GetPrimitiveArrayCritical(jviewMatrix, JNI_FALSE);

  b3ComputeViewMatrixFromPositions(cameraPosition, cameraTargetPosition, cameraUp, viewMatrix);

  env->ReleasePrimitiveArrayCritical(jviewMatrix, viewMatrix, 0);
  env->ReleasePrimitiveArrayCritical(jcameraUp, cameraUp, 0);
  env->ReleasePrimitiveArrayCritical(jcameraTargetPosition, cameraTargetPosition, 0);
  env->ReleasePrimitiveArrayCritical(jcameraPosition, cameraPosition, 0);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_computeViewMatrixFromYawPitchRoll
(JNIEnv *env, jobject obj, jfloatArray jcameraTargetPosition, jfloat distance,
 jfloat yaw, jfloat pitch, jfloat roll, jint upAxisIndex, jfloatArray jviewMatrix)
{
  
  float *cameraTargetPosition = NULL;
  float *viewMatrix = NULL;
  CHECKVALUE(jcameraTargetPosition, "computeProjectionMatrix: cameraTargetPosition is null",);
  CHECKDIMS(jcameraTargetPosition, 3, "computeProjectionMatrix: cameraTargetPosition must have dimension 3",);
  CHECKVALUE(jviewMatrix, "computeProjectionMatrix: viewMatrix is null",);
  CHECKDIMS(jviewMatrix, 16, "computeProjectionMatrix: viewMatrix must have dimension 16",);

  viewMatrix = (float *)env->GetPrimitiveArrayCritical(jviewMatrix, JNI_FALSE);
  cameraTargetPosition = (float *)env->GetPrimitiveArrayCritical(jcameraTargetPosition, JNI_FALSE);

  b3ComputeViewMatrixFromYawPitchRoll(cameraTargetPosition, distance, yaw, pitch, roll, upAxisIndex, viewMatrix);

  env->ReleasePrimitiveArrayCritical(jcameraTargetPosition, cameraTargetPosition, 0);
  env->ReleasePrimitiveArrayCritical(jviewMatrix, viewMatrix, 0);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_computeProjectionMatrix
(JNIEnv *env, jobject obj, jfloat left, jfloat right, jfloat bottom, jfloat top,
 jfloat nearVal, jfloat farVal, jfloatArray jprojectionMatrix)
{
  float *projectionMatrix = NULL;
  CHECKVALUE(jprojectionMatrix, "computeProjectionMatrix: projectionMatrix is null",);
  CHECKDIMS(jprojectionMatrix, 16, "computeProjectionMatrix: projectionMatrix must have dimension 16",);

  projectionMatrix = (float *)env->GetPrimitiveArrayCritical(jprojectionMatrix, JNI_FALSE);

  b3ComputeProjectionMatrix(left, right, bottom, top, nearVal, farVal, projectionMatrix);

  env->ReleasePrimitiveArrayCritical(jprojectionMatrix, projectionMatrix, 0);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_computeProjectionMatrixFOV
(JNIEnv *env, jobject obj, jfloat fov, jfloat aspect, jfloat nearVal, jfloat farVal, jfloatArray jprojectionMatrix)
{
  float *projectionMatrix = NULL;
  CHECKVALUE(jprojectionMatrix, "computeProjectionMatrix: projectionMatrix is null",);
  CHECKDIMS(jprojectionMatrix, 16, "computeProjectionMatrix: projectionMatrix must have dimension 16",);

  projectionMatrix = (float *)env->GetPrimitiveArrayCritical(jprojectionMatrix, JNI_FALSE);

  b3ComputeProjectionMatrixFOV(fov, aspect, nearVal, farVal, projectionMatrix);

  env->ReleasePrimitiveArrayCritical(jprojectionMatrix, projectionMatrix, 0);
}

jboolean getCameraImageBasic
(JNIEnv *env, jobject jRoboSimAPI, jint width, jint height,
 jfloatArray jviewMatrix, jfloatArray jprojectionMatrix,
 jfloatArray jlightProjection, jfloatArray jlightColor,
 jfloat lightDistance, jint hasShadow,
 jfloat lightAmbientCoeff, jfloat lightDiffuseCoeff, jfloat lightSpecularCoeff,
 jint renderer, struct b3CameraImageData &cameraImage)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  float *viewMatrix = NULL;
  float *projectionMatrix = NULL;
  float *lightProjection = NULL;
  float *lightColor = NULL;
  
  if (jviewMatrix != NULL) {
    CHECKDIMS(jviewMatrix, 16, "getCameraImage: viewMatrix must have dimension 16", false);
    viewMatrix = (float *)env->GetPrimitiveArrayCritical(jviewMatrix, JNI_FALSE);
  }
  if (jprojectionMatrix != NULL) {
    CHECKDIMS(jprojectionMatrix, 16, "getCameraImage: projectionMatrix must have dimension 16", false);
    projectionMatrix = (float *)env->GetPrimitiveArrayCritical(jprojectionMatrix, JNI_FALSE);
  }
  if (jlightProjection != NULL) {
    CHECKDIMS(jlightProjection, 3, "getCameraImage: lightProjection must have dimension 3", false);
    lightProjection = (float *)env->GetPrimitiveArrayCritical(jlightProjection, JNI_FALSE);
  }
  if (jlightColor != NULL) {
    CHECKDIMS(jlightColor, 3, "getCameraImage: lightColor must have dimension 3", false);
    lightColor = (float *)env->GetPrimitiveArrayCritical(jlightColor, JNI_FALSE);
  }

  jboolean status = jrsa -> getCameraImage(cameraImage, width, height,
					   viewMatrix, projectionMatrix,
					   lightProjection, lightColor,
					   lightDistance, hasShadow,
					   lightAmbientCoeff, lightDiffuseCoeff, lightSpecularCoeff,
					   renderer);

  if (lightColor != NULL) env->ReleasePrimitiveArrayCritical(jlightColor, lightColor, 0);
  if (lightProjection != NULL) env->ReleasePrimitiveArrayCritical(jlightProjection, lightProjection, 0);
  if (projectionMatrix != NULL) env->ReleasePrimitiveArrayCritical(jprojectionMatrix, projectionMatrix, 0);
  if (viewMatrix != NULL) env->ReleasePrimitiveArrayCritical(jviewMatrix, viewMatrix, 0);
  return status;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_getCameraImage
(JNIEnv *env, jobject jRoboSimAPI, jint width, jint height,
 jfloatArray jviewMatrix, jfloatArray jprojectionMatrix,
 jfloatArray jlightProjection, jfloatArray jlightColor,
 jfloat lightDistance, jint hasShadow,
 jfloat lightAmbientCoeff, jfloat lightDiffuseCoeff, jfloat lightSpecularCoeff,
 jint renderer, jobject jcameraImage)
{
  struct b3CameraImageData cameraImage;
  jboolean status = getCameraImageBasic(env, jRoboSimAPI, width, height,
					jviewMatrix, jprojectionMatrix,
					jlightProjection, jlightColor,
					lightDistance, hasShadow,
					lightAmbientCoeff, lightDiffuseCoeff, lightSpecularCoeff,
					renderer, cameraImage);
  nativeCameraImageDataToJava(env, jcameraImage, cameraImage);
  return status;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_getCameraImageInts
(JNIEnv *env, jobject jRoboSimAPI, jint width, jint height,
 jfloatArray jviewMatrix, jfloatArray jprojectionMatrix,
 jfloatArray jlightProjection, jfloatArray jlightColor,
 jfloat lightDistance, jint hasShadow,
 jfloat lightAmbientCoeff, jfloat lightDiffuseCoeff, jfloat lightSpecularCoeff,
 jint renderer, jintArray jrgbColorData, jfloatArray jdepthValues, jintArray jsegmentationMaskValues)
{
  int i;
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  struct b3CameraImageData cameraImage;
  int *rgbColorData = NULL;
  float *depthValues = NULL;
  int *segmentationMaskValues = NULL;

  jboolean status = getCameraImageBasic(env, jRoboSimAPI, width, height,
					jviewMatrix, jprojectionMatrix,
					jlightProjection, jlightColor,
					lightDistance, hasShadow,
					lightAmbientCoeff, lightDiffuseCoeff, lightSpecularCoeff,
					renderer, cameraImage);

  int length = width * height;
  CHECKVALUE(jrgbColorData, "getCameraImageInts: rgb output array is null", false);
  CHECKDIMS(jrgbColorData, length, "getCameraImageInts: rgb output array dimension doesnt match image size", false);

  rgbColorData = (int *)env->GetPrimitiveArrayCritical(jrgbColorData, JNI_FALSE);
  const unsigned char *data = cameraImage.m_rgbColorData;
  for (i = 0; i < length; i++) {
    int ii = i << 2;
    int r = ((int)(data[ii])) & 0xff;
    int g = ((int)(data[ii+1])) & 0xff;
    int b = ((int)(data[ii+2])) & 0xff;
    int a = ((int)(data[ii+3])) & 0xff;
    rgbColorData[i] = b | (g << 8) | (r << 16) | (a << 24);
  }
  env->ReleasePrimitiveArrayCritical(jrgbColorData, rgbColorData, 0);

  if (jdepthValues != NULL) {
    CHECKDIMS(jdepthValues, length, "getCameraImageInts: depth output array dimension doesnt match image size", false);
    depthValues = (float *)env->GetPrimitiveArrayCritical(jdepthValues, JNI_FALSE);
    memcpy(depthValues, cameraImage.m_depthValues, length*sizeof(float));
    env->ReleasePrimitiveArrayCritical(jdepthValues, depthValues, 0);
  }

  if (jsegmentationMaskValues != NULL) {
    CHECKDIMS(jsegmentationMaskValues, length, "getCameraImageInts: segmentation output array dimension doesnt match image size", false);
    segmentationMaskValues = (int *)env->GetPrimitiveArrayCritical(jsegmentationMaskValues, JNI_FALSE);
    memcpy(segmentationMaskValues, cameraImage.m_segmentationMaskValues, length*sizeof(int));
    env->ReleasePrimitiveArrayCritical(jsegmentationMaskValues, segmentationMaskValues, 0);
  }

  return status;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_getCameraImageBytes
(JNIEnv *env, jobject jRoboSimAPI, jobject jcameraImage, jint width, jint height,
 jfloatArray jviewMatrix, jfloatArray jprojectionMatrix,
 jfloatArray jlightProjection, jfloatArray jlightColor,
 jfloat lightDistance, jint hasShadow,
 jfloat lightAmbientCoeff, jfloat lightDiffuseCoeff, jfloat lightSpecularCoeff,
 jint renderer, jbyteArray jrgbColorData, jfloatArray jdepthValues, jintArray jsegmentationMaskValues)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  struct b3CameraImageData cameraImage;
  char *rgbColorData = NULL;
  float *depthValues = NULL;
  int *segmentationMaskValues = NULL;

  jboolean status = getCameraImageBasic(env, jRoboSimAPI, width, height,
					jviewMatrix, jprojectionMatrix,
					jlightProjection, jlightColor,
					lightDistance, hasShadow,
					lightAmbientCoeff, lightDiffuseCoeff, lightSpecularCoeff,
					renderer, cameraImage);

  int length = width * height;
  CHECKVALUE(jrgbColorData, "getCameraImageInts: rgb output array is null", false);
  CHECKDIMS(jrgbColorData, length, "getCameraImageInts: rgb output array dimension doesnt match image size", false);

  rgbColorData = (char *)env->GetPrimitiveArrayCritical(jrgbColorData, JNI_FALSE);
  memcpy(rgbColorData, cameraImage.m_rgbColorData, length*4*sizeof(char));
  env->ReleasePrimitiveArrayCritical(jrgbColorData, rgbColorData, 0);

  if (jdepthValues != NULL) {
    CHECKDIMS(jdepthValues, length, "getCameraImageInts: depth output array dimension doesnt match image size", false);
    depthValues = (float *)env->GetPrimitiveArrayCritical(jdepthValues, JNI_FALSE);
    memcpy(depthValues, cameraImage.m_depthValues, length*sizeof(float));
    env->ReleasePrimitiveArrayCritical(jdepthValues, depthValues, 0);
  }

  if (jsegmentationMaskValues != NULL) {
    CHECKDIMS(jsegmentationMaskValues, length, "getCameraImageInts: segmentation output array dimension doesnt match image size", false);
    segmentationMaskValues = (int *)env->GetPrimitiveArrayCritical(jsegmentationMaskValues, JNI_FALSE);
    memcpy(segmentationMaskValues, cameraImage.m_segmentationMaskValues, length*sizeof(int));
    env->ReleasePrimitiveArrayCritical(jsegmentationMaskValues, segmentationMaskValues, 0);
  }

  return status;
}


JNIEXPORT jint Java_edu_berkeley_bid_Bullet_addUserDebugParameter
(JNIEnv *env, jobject jRoboSimAPI, jstring jparamName, jdouble rangeMin, jdouble rangeMax, jdouble startValue)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  char *paramName = (char *)(env->GetStringUTFChars(jparamName, 0));
  
  int iparam = jrsa -> addUserDebugParameter(paramName, rangeMin, rangeMax, startValue);

  env->ReleaseStringUTFChars(jparamName, paramName);
  return iparam;
}

JNIEXPORT jdouble Java_edu_berkeley_bid_Bullet_readUserDebugParameter
(JNIEnv *env, jobject jRoboSimAPI, jint itemUniqueId)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  
  double dvalue = jrsa -> readUserDebugParameter(itemUniqueId);

  return dvalue;
}

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_removeUserDebugItem
(JNIEnv *env, jobject jRoboSimAPI, jint itemUniqueId)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  
  bool status = jrsa -> removeUserDebugItem(itemUniqueId);

  return status;
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
  b3JointInfo *jointInfo = javaJointInfoToNative(env, min);
  nativeJointInfoToJava(env, mout, jointInfo);
  deleteJointInfo(jointInfo);
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
  b3JointStates2 m = javaJointStates2ToNative(env, min, numJoints);
  nativeJointStates2ToJava(env, mout, m, numJoints);
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

JNIEXPORT void Java_edu_berkeley_bid_Bullet_testCameraImageData
(JNIEnv *env, jobject obj, jobject min, jobject mout)
{
  struct b3CameraImageData m = javaCameraImageDataToNative(env, min);
  nativeCameraImageDataToJava(env, mout, m);
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_testDynamicsInfo
(JNIEnv *env, jobject obj, jobject min, jobject mout)
{
  b3DynamicsInfo dynamicsInfo = javaDynamicsInfoToNative(env, min);
  nativeDynamicsInfoToJava(env, mout, dynamicsInfo);
}

}

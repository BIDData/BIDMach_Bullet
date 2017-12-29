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
    fprintf(stderr, "Couldnt initialize native Quaternion fields");
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
    fprintf(stderr, "Couldnt initialize native Vector3 fields");
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

static void nativeJointInfoToJava(JNIEnv *env, jobject jv, struct b3JointInfo &jointInfo) {
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/JointInfo");
  jfieldID linkNameID = env->GetFieldID(clazz, "m_linkName", "L/java/lang/String;");
  jfieldID jointNameID = env->GetFieldID(clazz, "m_jointName", "L/java/lang/String;");
  jfieldID jointTypeID = env->GetFieldID(clazz, "m_jointType", "I");
  jfieldID qIndexID = env->GetFieldID(clazz, "m_qIndex", "I");
  jfieldID uIndexID = env->GetFieldID(clazz, "m_uIndex", "I");
  jfieldID jointIndexID = env->GetFieldID(clazz, "m_jointIndex", "I");
  jfieldID flagsID = env->GetFieldID(clazz, "m_flags", "I");
  jfieldID jointDampingID = env->GetFieldID(clazz, "m_jointDamping", "D");
  jfieldID jointFrictionID = env->GetFieldID(clazz, "m_jointFriction", "D");
  jfieldID jointLowerLimitID = env->GetFieldID(clazz, "m_jointLowerLimit", "D");
  jfieldID jointUpperLimitID = env->GetFieldID(clazz, "m_jointUpperLimit", "D");
  jfieldID jointMaxForceID = env->GetFieldID(clazz, "m_jointMaxForce", "D");
  jfieldID jointMaxVelocityID = env->GetFieldID(clazz, "m_jointMaxVelocity", "D");
  jfieldID parentFrameID = env->GetFieldID(clazz, "m_parentFrame", "[D");
  jfieldID childFrameID = env->GetFieldID(clazz, "m_childFrame", "[D");
  jfieldID jointAxisID = env->GetFieldID(clazz, "m_jointAxis", "[D");
  int i;

  jstring jlinkName = env->NewStringUTF(jointInfo.m_linkName);
  jstring jjointName = env->NewStringUTF(jointInfo.m_jointName);
      
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


static struct b3JointInfo javaJointInfoToNative(JNIEnv *env, jobject jv) {
  struct b3JointInfo jointInfo;
  jclass clazz = (jclass) env->FindClass("edu/berkeley/bid/bullet/JointInfo");
  jfieldID linkNameID = env->GetFieldID(clazz, "m_linkName", "L/java/lang/String;");
  jfieldID jointNameID = env->GetFieldID(clazz, "m_jointName", "L/java/lang/String;");
  jfieldID jointTypeID = env->GetFieldID(clazz, "m_jointType", "I");
  jfieldID qIndexID = env->GetFieldID(clazz, "m_qIndex", "I");
  jfieldID uIndexID = env->GetFieldID(clazz, "m_uIndex", "I");
  jfieldID jointIndexID = env->GetFieldID(clazz, "m_jointIndex", "I");
  jfieldID flagsID = env->GetFieldID(clazz, "m_flags", "I");
  jfieldID jointDampingID = env->GetFieldID(clazz, "m_jointDamping", "D");
  jfieldID jointFrictionID = env->GetFieldID(clazz, "m_jointFriction", "D");
  jfieldID jointLowerLimitID = env->GetFieldID(clazz, "m_jointLowerLimit", "D");
  jfieldID jointUpperLimitID = env->GetFieldID(clazz, "m_jointUpperLimit", "D");
  jfieldID jointMaxForceID = env->GetFieldID(clazz, "m_jointMaxForce", "D");
  jfieldID jointMaxVelocityID = env->GetFieldID(clazz, "m_jointMaxVelocity", "D");
  jfieldID parentFrameID = env->GetFieldID(clazz, "m_parentFrame", "[D");
  jfieldID childFrameID = env->GetFieldID(clazz, "m_childFrame", "[D");
  jfieldID jointAxisID = env->GetFieldID(clazz, "m_jointAxis", "[D");
  int i;

  jstring jLinkName = (jstring)env->GetObjectField(jv, linkNameID);
  jstring jJointName = (jstring)env->GetObjectField(jv, jointNameID);

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

JNIEXPORT void Java_edu_berkeley_bid_Bullet_disconnect
(JNIEnv *env, jobject jRoboSimAPI)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> disconnect();
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

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_canSubmitCommand
(JNIEnv *env, jobject jRoboSimAPI)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  return jrsa -> canSubmitCommand();
}

JNIEXPORT void Java_edu_berkeley_bid_Bullet_stepSimulation
(JNIEnv *env, jobject jRoboSimAPI)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> stepSimulation();
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

JNIEXPORT void Java_edu_berkeley_bid_Bullet_configureDebugVisualizer
(JNIEnv *env, jobject jRoboSimAPI, jint flags, jint enable)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  jrsa -> configureDebugVisualizer((b3ConfigureDebugVisualizerEnum)flags, enable);
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

JNIEXPORT jboolean Java_edu_berkeley_bid_Bullet_getBodyInfo
(JNIEnv *env, jobject jRoboSimAPI, jint  bodyUniqueId, jobject jBodyInfo)
{
  b3RobotSimulatorClientAPI *jrsa = getRobotSimulatorClientAPI(env, jRoboSimAPI);
  struct b3BodyInfo bodyInfo;
  bool status = jrsa -> getBodyInfo(bodyUniqueId, &bodyInfo);
  jstring jbaseName = env->NewStringUTF(bodyInfo.m_baseName);
  jstring jbodyName = env->NewStringUTF(bodyInfo.m_bodyName);
  jclass clazz = env->GetObjectClass(jBodyInfo);
  jfieldID baseNameField = env->GetFieldID(clazz, "m_baseName", "Ljava/lang/String;");
  jfieldID bodyNameField = env->GetFieldID(clazz, "m_bodyName", "Ljava/lang/String;");
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


}

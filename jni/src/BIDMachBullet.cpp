#include <jni.h>
#include <../examples/RobotSimulator/b3RobotSimulatorClientAPI.h>

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

static b3RobotSimulatorClientAPI * getRobotSimulatorClientAPI(JNIEnv *env, jobject jRoboSimAPI)
{
  jclass clazz = env->GetObjectClass(jRoboSimAPI);
  jfieldID handle_id = env->GetFieldID(clazz, "handle", "J");
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
    jqclass = (jclass) env->NewGlobalRef(env->FindClass("edu/berkeley/bid/Bullet/Quaternion"));
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
    jvclass = (jclass) env->NewGlobalRef(env->FindClass("edu/berkeley/bid/Bullet/Vector3"));
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

}

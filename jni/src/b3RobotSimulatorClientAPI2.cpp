#include <jni.h>
#include <../examples/RobotSimulator/b3RobotSimulatorClientAPI.h>
#include <../examples/SharedMemory/PhysicsClientC_API.h>
#include <string.h>


// JFC: This struct added here because its in b3RobotSimulatorClient.cpp but not in any header.

struct b3RobotSimulatorClientAPI_InternalData
{
	b3PhysicsClientHandle m_physicsClientHandle;
	struct GUIHelperInterface* m_guiHelper;

	b3RobotSimulatorClientAPI_InternalData()
		: m_physicsClientHandle(0),
		m_guiHelper(0)
	{
	}
};


static void scalarToDouble3(b3Scalar a[3], double b[3]) {
  for (int i = 0; i < 3; i++) {
    b[i] = a[i];
  }
}

static void scalarToDouble4(b3Scalar a[4], double b[4]) {
  for (int i = 0; i < 4; i++) {
    b[i] = a[i];
  }
}

bool b3RobotSimulatorClientAPI::getLinkState(int bodyUniqueId, int linkIndex, int computeLinkVelocity, int computeForwardKinematics, b3LinkState* linkState)
{
  if (!isConnected()) {
    b3Warning("Not connected");
    return false;
  }
  b3SharedMemoryCommandHandle command = b3RequestActualStateCommandInit(m_data->m_physicsClientHandle, bodyUniqueId);

  if (computeLinkVelocity) {
    b3RequestActualStateCommandComputeLinkVelocity(command, computeLinkVelocity);
  }

  if (computeForwardKinematics) {
    b3RequestActualStateCommandComputeForwardKinematics(command, computeForwardKinematics);
  }

  b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
  

  if (b3GetStatusType(statusHandle) == CMD_ACTUAL_STATE_UPDATE_COMPLETED) {
    b3GetLinkState(m_data->m_physicsClientHandle, statusHandle, linkIndex, linkState);
    return true;
  }
  return false;
}

bool b3RobotSimulatorClientAPI::getCameraImage(int width, int height, struct b3RobotSimulatorGetCameraImageArgs args, struct b3CameraImageData &imageData)
{
  if (!isConnected()) {
    b3Warning("Not connected");
    return false;
  }
  
  b3SharedMemoryCommandHandle command;

  command = b3InitRequestCameraImage(m_data->m_physicsClientHandle);
  
  b3RequestCameraImageSetPixelResolution(command, width, height);

  // Check and apply optional arguments
  if (args.m_viewMatrix && args.m_projectionMatrix) {
    b3RequestCameraImageSetCameraMatrices(command, args.m_viewMatrix, args.m_projectionMatrix);
  }

  if (args.m_lightDirection != NULL)	{
    b3RequestCameraImageSetLightDirection(command, args.m_lightDirection);
  }

  if (args.m_lightColor != NULL) {
    b3RequestCameraImageSetLightColor(command, args.m_lightColor);
  }

  if (args.m_lightDistance>=0) {
    b3RequestCameraImageSetLightDistance(command, args.m_lightDistance);
  }

  if (args.m_hasShadow>=0) {
    b3RequestCameraImageSetShadow(command, args.m_hasShadow);
  }

  if (args.m_lightAmbientCoeff>=0) {
    b3RequestCameraImageSetLightAmbientCoeff(command, args.m_lightAmbientCoeff);
  }

  if (args.m_lightDiffuseCoeff>=0) {
    b3RequestCameraImageSetLightDiffuseCoeff(command, args.m_lightDiffuseCoeff);
  }

  if (args.m_lightSpecularCoeff>=0) {
    b3RequestCameraImageSetLightSpecularCoeff(command, args.m_lightSpecularCoeff);
  }

  if (args.m_renderer>=0) {
    b3RequestCameraImageSelectRenderer(command, args.m_renderer);
  }

  // Actually retrieve the image
  if (b3CanSubmitCommand(m_data->m_physicsClientHandle)) {
    b3SharedMemoryStatusHandle statusHandle;
    int statusType;
    
    statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
    statusType = b3GetStatusType(statusHandle);
    if (statusType == CMD_CAMERA_IMAGE_COMPLETED) {
      b3GetCameraImageData(m_data->m_physicsClientHandle, &imageData);
    }
  } else {
    return false;
  }
  return true;
}

bool b3RobotSimulatorClientAPI::calculateInverseDynamics(int bodyUniqueId, double *jointPositions, double *jointVelocities,
							 double *jointAccelerations, double *jointForcesOutput) 
{
  if (!isConnected()) {
    b3Warning("Not connected");
    return false;
  }

  int numJoints = b3GetNumJoints(m_data->m_physicsClientHandle, bodyUniqueId);
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;
  b3SharedMemoryCommandHandle commandHandle = b3CalculateInverseDynamicsCommandInit(m_data->m_physicsClientHandle, bodyUniqueId, jointPositions,
										    jointVelocities, jointAccelerations);
  
  statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, commandHandle);

  statusType = b3GetStatusType(statusHandle);

  if (statusType == CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED) {
    int bodyUniqueId;
    int dofCount;

    b3GetStatusInverseDynamicsJointForces(statusHandle, &bodyUniqueId, &dofCount, 0);

    if (dofCount) {
      b3GetStatusInverseDynamicsJointForces(statusHandle, 0, 0,	jointForcesOutput);
      return true;
    }
  }
  return false;
}

int b3RobotSimulatorClientAPI::getNumBodies() const
{
  if (!isConnected()) {
    b3Warning("Not connected");
    return false;
  }
  return b3GetNumBodies(m_data->m_physicsClientHandle);
}

int b3RobotSimulatorClientAPI::getBodyUniqueId(int bodyId) const
{
  if (!isConnected()) {
    b3Warning("Not connected");
    return false;
  }
  return b3GetBodyUniqueId(m_data->m_physicsClientHandle, bodyId);
}

bool b3RobotSimulatorClientAPI::removeBody(int bodyUniqueId) 
{
  if (!isConnected()) {
    b3Warning("Not connected");
    return false;
  }
  
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;
  
  if (b3CanSubmitCommand(m_data->m_physicsClientHandle)) {
    statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, b3InitRemoveBodyCommand(m_data->m_physicsClientHandle, bodyUniqueId));
    statusType = b3GetStatusType(statusHandle);
    if (statusType == CMD_REMOVE_BODY_COMPLETED) {
      return true;
    } else {
      b3Warning("getDynamicsInfo did not complete");
      return false;
    }
  }
  b3Warning("removeBody could not submit command");
  return false;
}

bool b3RobotSimulatorClientAPI::getDynamicsInfo(int bodyUniqueId, int linkIndex, b3DynamicsInfo *dynamicsInfo) {
  if (!isConnected()) {
    b3Warning("Not connected");
    return false;
  }
  int status_type = 0;
  b3SharedMemoryCommandHandle cmd_handle;
  b3SharedMemoryStatusHandle status_handle;
  struct b3DynamicsInfo info;

  if (bodyUniqueId < 0) {
    b3Warning("getDynamicsInfo failed; invalid bodyUniqueId");
    return false;
  }
  if (linkIndex < -1) {
    b3Warning("getDynamicsInfo failed; invalid linkIndex");
    return false;
  }
  
  if (b3CanSubmitCommand(m_data->m_physicsClientHandle)) {
    cmd_handle = b3GetDynamicsInfoCommandInit(m_data->m_physicsClientHandle, bodyUniqueId, linkIndex);
    status_handle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, cmd_handle);
    status_type = b3GetStatusType(status_handle);
    if (status_type == CMD_GET_DYNAMICS_INFO_COMPLETED) {
      return true;
    } else {
      b3Warning("getDynamicsInfo did not complete");
      return false;
    }
  } 
  b3Warning("getDynamicsInfo could not submit command");
  return false;
}

bool b3RobotSimulatorClientAPI::changeDynamics(int bodyUniqueId, int linkIndex, struct b3RobotSimulatorChangeDynamicsArgs &args)
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected to physics server.");
    return false;
  }
  b3SharedMemoryCommandHandle command = b3InitChangeDynamicsInfo(sm);
  b3SharedMemoryStatusHandle statusHandle;
		
  if (args.m_mass >= 0) {
    b3ChangeDynamicsInfoSetMass(command, bodyUniqueId, linkIndex, args.m_mass);
  }

  if (args.m_lateralFriction >= 0) {
    b3ChangeDynamicsInfoSetLateralFriction(command, bodyUniqueId, linkIndex, args.m_lateralFriction);
  }
  
  if (args.m_spinningFriction>=0) {
    b3ChangeDynamicsInfoSetSpinningFriction(command, bodyUniqueId, linkIndex, args.m_spinningFriction);
  }

  if (args.m_rollingFriction>=0) {
    b3ChangeDynamicsInfoSetRollingFriction(command, bodyUniqueId, linkIndex, args.m_rollingFriction);
  }

  if (args.m_linearDamping>=0)	{
    b3ChangeDynamicsInfoSetLinearDamping(command, bodyUniqueId, args.m_linearDamping);
  }

  if (args.m_angularDamping>=0) {
    b3ChangeDynamicsInfoSetAngularDamping(command, bodyUniqueId, args.m_angularDamping);
  }

  if (args.m_restitution>=0) {
    b3ChangeDynamicsInfoSetRestitution(command, bodyUniqueId, linkIndex, args.m_restitution);
  }

  if (args.m_contactStiffness>=0 && args.m_contactDamping >=0) {
    b3ChangeDynamicsInfoSetContactStiffnessAndDamping(command, bodyUniqueId, linkIndex, args.m_contactStiffness, args.m_contactDamping);
  }

  if (args.m_frictionAnchor>=0) {
    b3ChangeDynamicsInfoSetFrictionAnchor(command, bodyUniqueId,linkIndex, args.m_frictionAnchor);
  }
  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
  return true;
}

int b3RobotSimulatorClientAPI::addUserDebugParameter(char * paramName, double rangeMin, double rangeMax, double startValue) {
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected to physics server.");
    return -1;
  }
  b3SharedMemoryCommandHandle commandHandle;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;
  
  commandHandle = b3InitUserDebugAddParameter(sm, paramName, rangeMin, rangeMax, startValue);
  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
  statusType = b3GetStatusType(statusHandle);

  if (statusType == CMD_USER_DEBUG_DRAW_COMPLETED) {
    int debugItemUniqueId = b3GetDebugItemUniqueId(statusHandle);
    return  debugItemUniqueId;
  }
  b3Warning("addUserDebugParameter failed.");
  return -1;
}

int b3RobotSimulatorClientAPI::addUserDebugText(char *text, double *posXYZ, struct b3RobotSimulatorAddUserDebugTextArgs & args)
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected to physics server.");
    return -1;
  }
  b3SharedMemoryCommandHandle commandHandle;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;
  
  commandHandle = b3InitUserDebugDrawAddText3D(sm, text, posXYZ, &args.m_colorRGB[0], args.m_size, args.m_lifeTime);

  if (args.m_parentObjectUniqueId>=0) {
    b3UserDebugItemSetParentObject(commandHandle, args.m_parentObjectUniqueId, args.m_parentLinkIndex);
  }

  if (args.m_flags & DEBUG_TEXT_HAS_ORIENTATION) {
    b3UserDebugTextSetOrientation(commandHandle, &args.m_textOrientation[0]);
  }

  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
  statusType = b3GetStatusType(statusHandle);

  if (statusType == CMD_USER_DEBUG_DRAW_COMPLETED) {
    int debugItemUniqueId = b3GetDebugItemUniqueId(statusHandle);
    return  debugItemUniqueId;
  }
  b3Warning("addUserDebugText3D failed.");
  return -1;
}

int b3RobotSimulatorClientAPI::addUserDebugText(char *text, b3Vector3 &posXYZ, struct b3RobotSimulatorAddUserDebugTextArgs & args)
{
  double dposXYZ[3];
  dposXYZ[0] = posXYZ.x;
  dposXYZ[1] = posXYZ.y;
  dposXYZ[2] = posXYZ.z;

  return addUserDebugText(text, &dposXYZ[0], args);
}
  
int b3RobotSimulatorClientAPI::addUserDebugLine(double *fromXYZ, double *toXYZ, struct b3RobotSimulatorAddUserDebugLineArgs & args)
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected to physics server.");
    return -1;
  }
  b3SharedMemoryCommandHandle commandHandle;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;

  commandHandle = b3InitUserDebugDrawAddLine3D(sm, fromXYZ, toXYZ, &args.m_colorRGB[0], args.m_lineWidth, args.m_lifeTime);

  if (args.m_parentObjectUniqueId>=0) {
    b3UserDebugItemSetParentObject(commandHandle, args.m_parentObjectUniqueId, args.m_parentLinkIndex);
  }

  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
  statusType = b3GetStatusType(statusHandle);

  if (statusType == CMD_USER_DEBUG_DRAW_COMPLETED) {
    int debugItemUniqueId = b3GetDebugItemUniqueId(statusHandle);
    return  debugItemUniqueId;
  }
  b3Warning("addUserDebugLine failed.");
  return -1;
}

int b3RobotSimulatorClientAPI::addUserDebugLine(b3Vector3 &fromXYZ, b3Vector3 &toXYZ, struct b3RobotSimulatorAddUserDebugLineArgs & args)
{
  double dfromXYZ[3];
  double dtoXYZ[3];
  dfromXYZ[0] = fromXYZ.x;
  dfromXYZ[1] = fromXYZ.y;
  dfromXYZ[2] = fromXYZ.z;

  dtoXYZ[0] = toXYZ.x;
  dtoXYZ[1] = toXYZ.y;
  dtoXYZ[2] = toXYZ.z;
  return addUserDebugLine(&dfromXYZ[0], &dtoXYZ[0], args);
}
  
double b3RobotSimulatorClientAPI::readUserDebugParameter(int itemUniqueId) {
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected to physics server.");
    return 0;
  }
  b3SharedMemoryCommandHandle commandHandle;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;

  commandHandle = b3InitUserDebugReadParameter(sm, itemUniqueId);
  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
  statusType = b3GetStatusType(statusHandle);

  if (statusType == CMD_USER_DEBUG_DRAW_PARAMETER_COMPLETED) {
    double paramValue = 0.f;
    int ok = b3GetStatusDebugParameterValue(statusHandle, &paramValue);
    if (ok) {
      return paramValue;
    } 
  }
  b3Warning("readUserDebugParameter failed.");
  return 0;
}

bool b3RobotSimulatorClientAPI::removeUserDebugItem(int itemUniqueId) {
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected to physics server.");
    return false;
  }
  b3SharedMemoryCommandHandle commandHandle;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;

  commandHandle = b3InitUserDebugDrawRemove(sm, itemUniqueId);
  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
  statusType = b3GetStatusType(statusHandle);
  return true;
}

bool b3RobotSimulatorClientAPI::setJointMotorControlArray(int bodyUniqueId, struct b3RobotSimulatorJointMotorArrayArgs &args)
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected to physics server.");
    return false;
  }
  int numJoints = b3GetNumJoints(sm, bodyUniqueId);

  b3SharedMemoryCommandHandle commandHandle;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;
  struct b3JointInfo info;

  commandHandle = b3JointControlCommandInit2(sm, bodyUniqueId, args.m_controlMode);
  
  for (int i=0;i<args.m_numControlledDofs;i++) {
    double targetVelocity = 0.0;
    double targetPosition = 0.0;
    double force = 100000.0;
    double kp = 0.1;
    double kd = 1.0;
    int jointIndex;

    if (args.m_jointIndices) {
      jointIndex = args.m_jointIndices[i];
    } else {
      jointIndex = i;
    }

    if (args.m_targetVelocities) {
      targetVelocity = args.m_targetVelocities[i];
    }

    if (args.m_targetPositions) {
      targetPosition = args.m_targetPositions[i];
    }

    if (args.m_forces) {
      force = args.m_forces[i];
    }
			
    if (args.m_kps) {
      kp = args.m_kps[i];
    }
			
    if (args.m_kds) {
      kd = args.m_kds[i];
    }

    b3GetJointInfo(sm, bodyUniqueId, jointIndex, &info);
			
    switch (args.m_controlMode) {
    case CONTROL_MODE_VELOCITY: {
      b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex, targetVelocity);
      b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
      b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
      break;
    }

    case CONTROL_MODE_TORQUE: {
      b3JointControlSetDesiredForceTorque(commandHandle, info.m_uIndex, force);
      break;
    }

    case CONTROL_MODE_POSITION_VELOCITY_PD: {
      b3JointControlSetDesiredPosition(commandHandle, info.m_qIndex, targetPosition);
      b3JointControlSetKp(commandHandle, info.m_uIndex, kp);
      b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex, targetVelocity);
      b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
      b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
      break;
    }

    default: {}
    };
  }
  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
  return true;
}


bool b3RobotSimulatorClientAPI::setPhysicsEngineParameter(struct b3RobotSimulatorSetPhysicsEngineParameters &args)
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected");
    return false;
  }
  b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
  b3SharedMemoryStatusHandle statusHandle;

  if (args.m_numSolverIterations >= 0) {
      b3PhysicsParamSetNumSolverIterations(command, args.m_numSolverIterations);
  }

  if (args.m_collisionFilterMode >= 0) {
    b3PhysicsParamSetCollisionFilterMode(command, args.m_collisionFilterMode);
  }

  if (args.m_numSubSteps >= 0)	{
    b3PhysicsParamSetNumSubSteps(command, args.m_numSubSteps);
  }

  if (args.m_fixedTimeStep >= 0) {
    b3PhysicsParamSetTimeStep(command, args.m_fixedTimeStep);
  }

  if (args.m_useSplitImpulse >= 0) {
    b3PhysicsParamSetUseSplitImpulse(command, args.m_useSplitImpulse);
  }
  
  if (args.m_splitImpulsePenetrationThreshold >= 0) {
    b3PhysicsParamSetSplitImpulsePenetrationThreshold(command, args.m_splitImpulsePenetrationThreshold);
  }
  
  if (args.m_contactBreakingThreshold >= 0) {
    b3PhysicsParamSetContactBreakingThreshold(command, args.m_contactBreakingThreshold);
  }

  if (args.m_maxNumCmdPer1ms >= -1) {
    b3PhysicsParamSetMaxNumCommandsPer1ms(command, args.m_maxNumCmdPer1ms);
  }

  if (args.m_restitutionVelocityThreshold>=0) {
    b3PhysicsParamSetRestitutionVelocityThreshold(command, args.m_restitutionVelocityThreshold);
  }

  if (args.m_enableFileCaching>=0) {
    b3PhysicsParamSetEnableFileCaching(command, args.m_enableFileCaching);
  }
  
  if (args.m_erp>=0) {
    b3PhysicsParamSetDefaultNonContactERP(command,args.m_erp);
  }

  if (args.m_contactERP>=0) {
    b3PhysicsParamSetDefaultContactERP(command,args.m_contactERP);
  }
  
  if (args.m_frictionERP >=0) {
    b3PhysicsParamSetDefaultFrictionERP(command,args.m_frictionERP);
  }

  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
  return true;
}

bool b3RobotSimulatorClientAPI::applyExternalForce(int objectUniqueId, int linkIndex, double *force, double *position, int flags) 
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected");
    return false;
  }
  b3SharedMemoryCommandHandle command;
  b3SharedMemoryStatusHandle statusHandle;

  command = b3ApplyExternalForceCommandInit(sm);
  b3ApplyExternalForce(command, objectUniqueId, linkIndex, force, position, flags);
  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
  return true;
}

bool b3RobotSimulatorClientAPI::applyExternalForce(int objectUniqueId, int linkIndex, b3Vector3 &force, b3Vector3 &position, int flags)
{
  double dforce[3];
  double dposition[3];

  dforce[0] = force.x;
  dforce[1] = force.y;
  dforce[2] = force.z;

  dposition[0] = position.x;
  dposition[1] = position.y;
  dposition[2] = position.z;

  return applyExternalForce(objectUniqueId, linkIndex, &dforce[0], &dposition[0], flags);
}


bool b3RobotSimulatorClientAPI::applyExternalTorque(int objectUniqueId, int linkIndex, double *torque, int flags) 
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected");
    return false;
  }
  b3SharedMemoryCommandHandle command;
  b3SharedMemoryStatusHandle statusHandle;

  command = b3ApplyExternalForceCommandInit(sm);
  b3ApplyExternalTorque(command, objectUniqueId, linkIndex, torque, flags);
  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
  return true;
}

bool b3RobotSimulatorClientAPI::applyExternalTorque(int objectUniqueId, int linkIndex, b3Vector3 &torque, int flags)
{
  double dtorque[3];

  dtorque[0] = torque.x;
  dtorque[1] = torque.y;
  dtorque[2] = torque.z;

  return applyExternalTorque(objectUniqueId, linkIndex, &dtorque[0], flags);
}

bool  b3RobotSimulatorClientAPI::enableJointForceTorqueSensor(int bodyUniqueId, int jointIndex, bool enable)
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected");
    return false;
  }
  int numJoints = b3GetNumJoints(sm, bodyUniqueId);
  if ((jointIndex < 0) || (jointIndex >= numJoints)) {
      b3Warning("Error: invalid jointIndex.");
      return false;
  }
  b3SharedMemoryCommandHandle command;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;
		
  command = b3CreateSensorCommandInit(sm, bodyUniqueId);
  b3CreateSensorEnable6DofJointForceTorqueSensor(command, jointIndex, enable);
  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
  statusType = b3GetStatusType(statusHandle);
  if (statusType == CMD_CLIENT_COMMAND_COMPLETED) {
    return true;
  }
  return false;
}

bool b3RobotSimulatorClientAPI::getDebugVisualizerCamera(struct b3OpenGLVisualizerCameraInfo *cameraInfo)
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected");
    return false;
  }
  b3SharedMemoryCommandHandle command;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;

  command = b3InitRequestOpenGLVisualizerCameraCommand(sm);
  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
  statusType = b3GetStatusOpenGLVisualizerCamera(statusHandle, cameraInfo);

  if (statusType) {
    return true;
  }
  return false;
}

bool b3RobotSimulatorClientAPI::getContactPoints(struct b3RobotSimulatorGetContactPointsArgs &args, struct b3ContactInformation *contactInfo)
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected");
    return false;
  }
  b3SharedMemoryCommandHandle command;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;

  command = b3InitRequestContactPointInformation(sm);

  if (args.m_bodyUniqueIdA>=0) {
    b3SetContactFilterBodyA(command, args.m_bodyUniqueIdA);
  }
  if (args.m_bodyUniqueIdB>=0) {
    b3SetContactFilterBodyB(command, args.m_bodyUniqueIdB);
  }
  if (args.m_linkIndexA>=-1) {
    b3SetContactFilterLinkA(command, args.m_linkIndexA);
  }
  if (args.m_linkIndexB >=-1) {
    b3SetContactFilterLinkB(command, args.m_linkIndexB);
  }

  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
  statusType = b3GetStatusType(statusHandle);

  if (statusType == CMD_CONTACT_POINT_INFORMATION_COMPLETED) {
    b3GetContactPointInformation(sm, contactInfo);
    return true;
  }
  return false;
}

bool b3RobotSimulatorClientAPI::getClosestPoints(struct b3RobotSimulatorGetContactPointsArgs &args, double distance, struct b3ContactInformation *contactInfo)
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected");
    return false;
  }
  b3SharedMemoryCommandHandle command;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;

  command = b3InitClosestDistanceQuery(sm);

  b3SetClosestDistanceFilterBodyA(command, args.m_bodyUniqueIdA);
  b3SetClosestDistanceFilterBodyB(command, args.m_bodyUniqueIdB);
  b3SetClosestDistanceThreshold(command, distance);

  if (args.m_linkIndexA>=-1) {
    b3SetClosestDistanceFilterLinkA(command, args.m_linkIndexA);
  }
  if (args.m_linkIndexB >=-1) {
    b3SetClosestDistanceFilterLinkB(command, args.m_linkIndexB);
  }

  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
  statusType = b3GetStatusType(statusHandle);

  if (statusType == CMD_CONTACT_POINT_INFORMATION_COMPLETED) {
    b3GetContactPointInformation(sm, contactInfo);
    return true;
  }
  return false;
}

bool b3RobotSimulatorClientAPI::getOverlappingObjects(double *aabbMin, double *aabbMax, struct b3AABBOverlapData *overlapData)
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected");
    return false;
  }
  b3SharedMemoryCommandHandle command;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;

  command = b3InitAABBOverlapQuery(sm, aabbMin, aabbMax);
  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
  b3GetAABBOverlapResults(sm, overlapData);

  return true;
}


bool b3RobotSimulatorClientAPI::getOverlappingObjects(b3Vector3 &aabbMin, b3Vector3 &aabbMax, struct b3AABBOverlapData *overlapData)
{
  double daabbMin[3];
  double daabbMax[3];

  daabbMin[0] = aabbMin.x;
  daabbMin[1] = aabbMin.y;
  daabbMin[2] = aabbMin.z;

  daabbMax[0] = aabbMax.x;
  daabbMax[1] = aabbMax.y;
  daabbMax[2] = aabbMax.z;

  return getOverlappingObjects(&daabbMin[0], &daabbMax[0], overlapData);
}


  
bool b3RobotSimulatorClientAPI::getAABB(int bodyUniqueId, int linkIndex, double *aabbMin, double *aabbMax)
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected");
    return false;
  }
  b3SharedMemoryCommandHandle command;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;

  if (bodyUniqueId < 0) {
    b3Warning("Invalid bodyUniqueId");
    return false;
  }

  if (linkIndex < -1) {
    b3Warning("Invalid linkIndex");
    return false;
  }

  if (aabbMin == NULL || aabbMax == NULL) {
    b3Warning("Output AABB matrix is NULL");
    return false;
  }    

  command = b3RequestCollisionInfoCommandInit(sm, bodyUniqueId);
  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
  
  statusType = b3GetStatusType(statusHandle);
  if (statusType != CMD_REQUEST_COLLISION_INFO_COMPLETED) {
    return false;
  }
  if (b3GetStatusAABB(statusHandle, linkIndex, aabbMin, aabbMax)) {
    return true;
  }
  return false;
}

bool b3RobotSimulatorClientAPI::getAABB(int bodyUniqueId, int linkIndex, b3Vector3 &aabbMin, b3Vector3 &aabbMax)
{
  double daabbMin[3];
  double daabbMax[3];

  bool status = getAABB(bodyUniqueId, linkIndex, &daabbMin[0], &daabbMax[0]);

  aabbMin.x = (float)daabbMin[0];
  aabbMin.y = (float)daabbMin[1];
  aabbMin.z = (float)daabbMin[2];

  aabbMax.x = (float)daabbMax[0];
  aabbMax.y = (float)daabbMax[1];
  aabbMax.z = (float)daabbMax[2];

  return status;
}


void b3RobotSimulatorClientAPI::getMouseEvents(b3MouseEventsData* mouseEventsData)
{
  mouseEventsData->m_numMouseEvents = 0;
  mouseEventsData->m_mouseEvents = 0;
  if (!isConnected()) {
    b3Warning("Not connected");
    return;
  }

  b3SharedMemoryCommandHandle command = b3RequestMouseEventsCommandInit(m_data->m_physicsClientHandle);
  b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClientHandle, command);
  b3GetMouseEventsData(m_data->m_physicsClientHandle, mouseEventsData);
}

int b3RobotSimulatorClientAPI::createCollisionShape(int shapeType, struct b3RobotSimulatorCreateCollisionShapeArgs &args)
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected");
    return false;
  }
  b3SharedMemoryCommandHandle command;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;
  int shapeIndex = -1;

  command = b3CreateCollisionShapeCommandInit(sm);

  if (shapeType==GEOM_SPHERE && args.m_radius>0) {
    shapeIndex = b3CreateCollisionShapeAddSphere(command, args.m_radius);
  }
  if (shapeType==GEOM_BOX)  {
    double halfExtents[3];
    scalarToDouble3(args.m_halfExtents, halfExtents);
    shapeIndex = b3CreateCollisionShapeAddBox(command, halfExtents);
  }
  if (shapeType==GEOM_CAPSULE && args.m_radius>0 && args.m_height>=0) {
    shapeIndex = b3CreateCollisionShapeAddCapsule(command, args.m_radius, args.m_height);
  }
  if (shapeType==GEOM_CYLINDER && args.m_radius>0 && args.m_height>=0) {
    shapeIndex = b3CreateCollisionShapeAddCylinder(command, args.m_radius, args.m_height);
  }
  if (shapeType==GEOM_MESH && args.m_fileName) {
    double meshScale[3];
    scalarToDouble3(args.m_meshScale, meshScale);
    shapeIndex = b3CreateCollisionShapeAddMesh(command, args.m_fileName, meshScale);
  }
  if (shapeType==GEOM_PLANE) {
    double planeConstant=0;
    double planeNormal[3];
    scalarToDouble3(args.m_planeNormal, planeNormal);
    shapeIndex = b3CreateCollisionShapeAddPlane(command, planeNormal, planeConstant);
  }
  if (shapeIndex>=0 && args.m_flags) {
    b3CreateCollisionSetFlag(command, shapeIndex, args.m_flags);
  }

  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
  statusType = b3GetStatusType(statusHandle);
  if (statusType == CMD_CREATE_COLLISION_SHAPE_COMPLETED) {
    int uid = b3GetStatusCollisionShapeUniqueId(statusHandle);
    return uid;
  }
  return -1;
}

int b3RobotSimulatorClientAPI::createMultiBody(struct b3RobotSimulatorCreateMultiBodyArgs &args)
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected");
    return false;
  }
  b3SharedMemoryCommandHandle command;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType, baseIndex;

  double doubleBasePosition[3];
  double doubleBaseInertialFramePosition[3];
  scalarToDouble3(args.m_basePosition.m_floats, doubleBasePosition);
  scalarToDouble3(args.m_baseInertialFramePosition.m_floats, doubleBaseInertialFramePosition);

  double doubleBaseOrientation[4];
  double doubleBaseInertialFrameOrientation[4];
  scalarToDouble4(args.m_baseOrientation.m_floats, doubleBaseOrientation);
  scalarToDouble4(args.m_baseInertialFrameOrientation.m_floats, doubleBaseInertialFrameOrientation);

  command = b3CreateMultiBodyCommandInit(sm);

  baseIndex = b3CreateMultiBodyBase(command, args.m_baseMass, args.m_baseCollisionShapeIndex, args.m_baseVisualShapeIndex,
				    doubleBasePosition, doubleBaseOrientation, doubleBaseInertialFramePosition, doubleBaseInertialFrameOrientation);

  for (int i = 0; i < args.m_numLinks; i++) {
    double linkMass = args.m_linkMasses[i];
    int linkCollisionShapeIndex = args.m_linkCollisionShapeIndices[i];
    int linkVisualShapeIndex = args.m_linkVisualShapeIndices[i];
    b3Vector3 linkPosition = args.m_linkPositions[i];
    b3Quaternion linkOrientation = args.m_linkOrientations[i];
    b3Vector3 linkInertialFramePosition = args.m_linkInertialFramePositions[i];
    b3Quaternion linkInertialFrameOrientation = args.m_linkInertialFrameOrientations[i];
    int linkParentIndex = args.m_linkParentIndices[i];
    int linkJointType = args.m_linkJointTypes[i];
    b3Vector3 linkJointAxis = args.m_linkJointAxes[i];

    double doubleLinkPosition[3];
    double doubleLinkInertialFramePosition[3];
    double doubleLinkJointAxis[3];
    scalarToDouble3(linkPosition.m_floats, doubleLinkPosition);
    scalarToDouble3(linkInertialFramePosition.m_floats, doubleLinkInertialFramePosition);
    scalarToDouble3(linkJointAxis.m_floats, doubleLinkJointAxis);
    
    double doubleLinkOrientation[4];
    double doubleLinkInertialFrameOrientation[4];
    scalarToDouble4(linkOrientation.m_floats, doubleLinkOrientation);
    scalarToDouble4(linkInertialFrameOrientation.m_floats, doubleLinkInertialFrameOrientation);
    
    b3CreateMultiBodyLink(command, 
			  linkMass, 
			  linkCollisionShapeIndex, 
			  linkVisualShapeIndex, 
			  doubleLinkPosition, 
			  doubleLinkOrientation,
			  doubleLinkInertialFramePosition,
			  doubleLinkInertialFrameOrientation,
			  linkParentIndex,
			  linkJointType,
			  doubleLinkJointAxis
			  );    
  }

  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
  statusType = b3GetStatusType(statusHandle);
  if (statusType == CMD_CREATE_MULTI_BODY_COMPLETED) {
    int uid = b3GetStatusBodyIndex(statusHandle);
    return uid;
  }
  return -1;
}

int b3RobotSimulatorClientAPI::getNumConstraints() const
{
  if (!isConnected()) {
    b3Warning("Not connected");
    return -1;
  }
  return b3GetNumUserConstraints(m_data->m_physicsClientHandle);
}

int b3RobotSimulatorClientAPI::getConstraintUniqueId(int serialIndex)
{
  if (!isConnected()) {
    b3Warning("Not connected");
    return -1;
  }
  int userConstraintId = -1;
  userConstraintId = b3GetUserConstraintId(m_data->m_physicsClientHandle, serialIndex);
  return userConstraintId;
}


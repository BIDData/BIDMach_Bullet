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
    b3ChangeDynamicsInfoSetSpinningFriction(command, bodyUniqueId, linkIndex,args.m_spinningFriction);
  }

  if (args.m_rollingFriction>=0) {
    b3ChangeDynamicsInfoSetRollingFriction(command, bodyUniqueId, linkIndex,args.m_rollingFriction);
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

int b3RobotSimulatorClientAPI::addUserDebugText3D(char *text, double *posXYZ, struct b3RobotSimulatorAddUserDebugText3DArgs & args)
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

  b3UserDebugTextSetOrientation(commandHandle, &args.m_textOrientation[0]);

  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
  statusType = b3GetStatusType(statusHandle);

  if (statusType == CMD_USER_DEBUG_DRAW_COMPLETED) {
    int debugItemUniqueId = b3GetDebugItemUniqueId(statusHandle);
    return  debugItemUniqueId;
  }
  b3Warning("addUserDebugText3D failed.");
  return -1;
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









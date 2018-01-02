#include <jni.h>
#include <../examples/RobotSimulator/b3RobotSimulatorClientAPI.h>
#include <../examples/SharedMemory/PhysicsClientC_API.h>
#include <string.h>

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


bool b3RobotSimulatorClientAPI::getCameraImage(struct b3CameraImageData &imageData, int width, int height,
					       float *viewMatrix, float *projectionMatrix,
					       float *lightDirection, float *lightColor,
					       float lightDistance, int hasShadow,
					       float lightAmbientCoeff, float lightDiffuseCoeff, float lightSpecularCoeff,
					       int renderer)
{
  if (!isConnected()) {
    b3Warning("Not connected");
    return false;
  }
  
  b3SharedMemoryCommandHandle command;

  command = b3InitRequestCameraImage(m_data->m_physicsClientHandle);
  
  b3RequestCameraImageSetPixelResolution(command, width, height);

  // Check and apply optional arguments
  if (viewMatrix && projectionMatrix) {
    b3RequestCameraImageSetCameraMatrices(command, viewMatrix, projectionMatrix);
  }

  if (lightDirection)	{
    b3RequestCameraImageSetLightDirection(command, lightDirection);
  }

  if (lightColor) {
    b3RequestCameraImageSetLightColor(command, lightColor);
  }

  if (lightDistance>=0) {
    b3RequestCameraImageSetLightDistance(command, lightDistance);
  }

  if (hasShadow>=0) {
    b3RequestCameraImageSetShadow(command, hasShadow);
  }

  if (lightAmbientCoeff>=0) {
    b3RequestCameraImageSetLightAmbientCoeff(command, lightAmbientCoeff);
  }

  if (lightDiffuseCoeff>=0) {
    b3RequestCameraImageSetLightDiffuseCoeff(command, lightDiffuseCoeff);
  }

  if (lightSpecularCoeff>=0) {
    b3RequestCameraImageSetLightSpecularCoeff(command, lightSpecularCoeff);
  }

  if (renderer>=0) {
    b3RequestCameraImageSelectRenderer(command, renderer);
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

bool b3RobotSimulatorClientAPI::changeDynamics(int bodyUniqueId, int linkIndex,	double mass, double lateralFriction, double spinningFriction,
					       double rollingFriction, double restitution, double linearDamping, double angularDamping,
					       double contactStiffness, double contactDamping, int frictionAnchor)
{
  b3PhysicsClientHandle sm = m_data->m_physicsClientHandle;
  if (sm == 0) {
    b3Warning("Not connected to physics server.");
    return false;
  }
  b3SharedMemoryCommandHandle command = b3InitChangeDynamicsInfo(sm);
  b3SharedMemoryStatusHandle statusHandle;
		
  if (mass >= 0) {
    b3ChangeDynamicsInfoSetMass(command, bodyUniqueId, linkIndex, mass);
  }

  if (lateralFriction >= 0) {
    b3ChangeDynamicsInfoSetLateralFriction(command, bodyUniqueId, linkIndex, lateralFriction);
  }
  
  if (spinningFriction>=0) {
    b3ChangeDynamicsInfoSetSpinningFriction(command, bodyUniqueId, linkIndex,spinningFriction);
  }

  if (rollingFriction>=0) {
    b3ChangeDynamicsInfoSetRollingFriction(command, bodyUniqueId, linkIndex,rollingFriction);
  }

  if (linearDamping>=0)	{
    b3ChangeDynamicsInfoSetLinearDamping(command,bodyUniqueId, linearDamping);
  }

  if (angularDamping>=0) {
    b3ChangeDynamicsInfoSetAngularDamping(command,bodyUniqueId,angularDamping);
  }

  if (restitution>=0) {
    b3ChangeDynamicsInfoSetRestitution(command, bodyUniqueId, linkIndex, restitution);
  }

  if (contactStiffness>=0 && contactDamping >=0) {
    b3ChangeDynamicsInfoSetContactStiffnessAndDamping(command,bodyUniqueId,linkIndex,contactStiffness, contactDamping);
  }

  if (frictionAnchor>=0) {
    b3ChangeDynamicsInfoSetFrictionAnchor(command,bodyUniqueId,linkIndex, frictionAnchor);
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



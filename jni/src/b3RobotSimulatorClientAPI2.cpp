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

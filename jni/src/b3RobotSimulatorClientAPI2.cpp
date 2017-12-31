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


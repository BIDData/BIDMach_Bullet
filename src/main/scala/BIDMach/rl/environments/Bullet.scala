package BIDMach.rl.environments;
import BIDMach.rl.environments.bullet._;
import edu.berkeley.bid.bullet.{Vector3,Matrix3x3}
import BIDMat.{BMat,DMat,FMat,IMat,Quaternion};
import BIDMat.MatFunctions;


class Bullet {

    val javaBullet = new edu.berkeley.bid.Bullet();

    import BIDMach.rl.environments.Bullet._;

    var filePathPrefix:String = null;

    def appendPathPrefix(fname:String):String = {
	if (filePathPrefix.asInstanceOf[AnyRef] != null) {
	    filePathPrefix + fname;
	} else {
	    fname;
	}
    };

    def setPathPrefix(dirname:String):String = {
	filePathPrefix = dirname;
	dirname;
    };

    def connect(method:Int, hostname:String="localhost", port:Int= -1):Boolean = {
	javaBullet.connect(method, hostname, port);
    };

    def disconnect():Unit = {
	javaBullet.disconnect();
    };
	
    def isConnected():Boolean = {
	javaBullet.isConnected();
    };

    def setGravity(gravity:FMat):Unit = {
	val gravity0 = fromFMatToVector3(gravity);
	javaBullet.setGravity(gravity0);
    };

    def setGravity(x:Float, y:Float, z:Float):Unit = {
	val gravity = new Vector3(x, y, z);
	javaBullet.setGravity(gravity);
    };

    def loadURDF(fname:String, startPos:FMat=null, startOrient:Quaternion=null, forceOverrideFixedBase:Boolean=false, useMultiBody:Boolean=true, flags:Int=0):Int = {
	val startPos0 = fromFMatToVector3(startPos);
	val startOrient0 = BIDMatQtoJavaQ(startOrient);
	javaBullet.loadURDF(appendPathPrefix(fname), startPos0, startOrient0, forceOverrideFixedBase, useMultiBody, flags);
    };

    def loadSDF(fname:String, forceOverrideFixedBase:Boolean=false, useMultiBody:Boolean=true):IMat = {
	val ints:Array[Int] = javaBullet.loadSDF(appendPathPrefix(fname), forceOverrideFixedBase, useMultiBody);
	MatFunctions.irow(ints);
    };

    def loadMJCF(fname:String):IMat = {
	val ints:Array[Int] = javaBullet.loadMJCF(appendPathPrefix(fname));
	MatFunctions.irow(ints);
    };

    def loadBullet(fname:String):IMat = {
	val ints:Array[Int] = javaBullet.loadBullet(appendPathPrefix(fname));
	MatFunctions.irow(ints);
    };

    def stepSimulation():Unit = {
	javaBullet.stepSimulation();
    };

    def setRealTimeSimulation(enable:Boolean) = {
	javaBullet.setRealTimeSimulation(enable);
    };

    def getBasePositionAndOrientation(bodyUniqueId:Int):(FMat, Quaternion) = {
	val basePosition = new Vector3();
	val baseOrientation = new edu.berkeley.bid.bullet.Quaternion();
	javaBullet.getBasePositionAndOrientation(bodyUniqueId, basePosition, baseOrientation);
	(fromVector3ToFMat(basePosition), JavaQtoBIDMatQ(baseOrientation));
    };

    def resetBasePositionAndOrientation(bodyUniqueId:Int, position:FMat, orientation:Quaternion) = {
	val basePosition = fromFMatToVector3(position);
	val baseOrientation = BIDMatQtoJavaQ(orientation);
	javaBullet.resetBasePositionAndOrientation(bodyUniqueId, basePosition, baseOrientation);
    };

    def getQuaternionFromEuler(euler:FMat):BIDMat.Quaternion = {
	val euler0 = fromFMatToVector3(euler);
	val q = new edu.berkeley.bid.bullet.Quaternion();
	edu.berkeley.bid.Bullet.getQuaternionFromEuler(euler0, q);
	JavaQtoBIDMatQ(q);
    };

    def getQuaternionFromEuler(yawZ:Double, pitchY:Double, rollX:Double):BIDMat.Quaternion = {
	val euler0 = fromFMatToVector3(MatFunctions.row(yawZ, pitchY, rollX));
	val q = new edu.berkeley.bid.bullet.Quaternion();
	edu.berkeley.bid.Bullet.getQuaternionFromEuler(euler0, q);
	JavaQtoBIDMatQ(q);
    };

    def getEulerFromQuaternion(q:BIDMat.Quaternion):FMat = {
	val v = new Vector3();
	val q0 = BIDMatQtoJavaQ(q);
	edu.berkeley.bid.Bullet.getEulerFromQuaternion(q0, v);
	fromVector3ToFMat(v);
    }

    def getNumJoints(bodyUniqueId:Int):Int = {
	javaBullet.getNumJoints(bodyUniqueId);
    };

    def getJointInfo(bodyUniqueId:Int, jointIndex:Int):JointInfo = {
	val jointInfo = new JointInfo();
	javaBullet.getJointInfo(bodyUniqueId, jointIndex, jointInfo.javaJointInfo);
	jointInfo;
    };

    def setJointMotorControl(bodyUniqueId:Int, jointIndex:Int, controlMode:Int,
			     targetPosition:Double = 0, targetVelocity:Double = 0,
			     force:Double = 100000, positionGain:Double = 0.1, velocityGain:Double = 1.0):Unit = {
	javaBullet.setJointMotorControl(bodyUniqueId, jointIndex, controlMode, targetPosition, targetVelocity, force, positionGain, velocityGain);
    };

    def setJointMotorControlArray(bodyUniqueId:Int, jointIndices:IMat, controlMode:Int,
				  targetPositions:DMat = null, targetVelocities:DMat = null, forces:DMat = null,
				  positionGains:DMat = null, velocityGains:DMat = null):Boolean = {
	javaBullet.setJointMotorControlArray(bodyUniqueId, jointIndices.data, controlMode,
					     getData(targetPositions), getData(targetVelocities), getData(forces),
					     getData(positionGains), getData(velocityGains));
    };

    def getJointState(bodyUniqueId:Int, jointIndex:Int):JointSensorState = {
	val jointState = new JointSensorState();
	javaBullet.getJointState(bodyUniqueId, jointIndex, jointState.javaSensorState);
	jointState;
    };

    def resetJointState(bodyUniqueId:Int, jointIndex:Int, targetValue:Double):Boolean = {
	javaBullet.resetJointState(bodyUniqueId, jointIndex, targetValue);
    };

    def getJointStates(bodyUniqueId:Int):JointStates = {
	val javaJointStates = new edu.berkeley.bid.bullet.JointStates2();
	javaBullet.getJointStates(bodyUniqueId, javaJointStates);
	new JointStates(javaJointStates);
    };    

    def getJointStates4(bodyUniqueId:Int, jointIndices:IMat):(DMat, DMat, DMat, DMat) = {
	val numJoints = jointIndices.length;
	val jointPositions = DMat.zeros(1, numJoints);
	val jointVelocities = DMat.zeros(1, numJoints);
	val jointForceTorques = DMat.zeros(numJoints, 6);
	val jointMotorTorques = DMat.zeros(1, numJoints);
	javaBullet.getJointStates(bodyUniqueId, jointIndices.data,
				  jointPositions.data, jointVelocities.data,
				  jointForceTorques.data, jointMotorTorques.data);
	(jointPositions, jointVelocities, jointForceTorques.t, jointMotorTorques);
    };

    def enableJointForceTorqueSensor(bodyUniqueId:Int, jointIndex:Int, enable:Boolean) = {
	javaBullet.enableJointForceTorqueSensor(bodyUniqueId, jointIndex, enable);
    };

    def getLinkState(bodyUniqueId:Int, linkIndex:Int, computeLinkVelocity:Int = 0, computeForwardKinematics:Int = 0):LinkState = {
	val linkState = new LinkState();
	javaBullet.getLinkState(bodyUniqueId, linkIndex, computeLinkVelocity, computeForwardKinematics, linkState.javaLinkState);
	linkState;
    };

    def getBaseVelocity(bodyUniqueId:Int):(FMat, FMat) = {
	val baseVelocity = new Vector3();
	val baseAngularVelocity = new Vector3
	javaBullet.getBaseVelocity(bodyUniqueId, baseVelocity, baseAngularVelocity);
	(fromVector3ToFMat(baseVelocity), fromVector3ToFMat(baseAngularVelocity));
    };

    def resetBaseVelocity(bodyUniqueId:Int, baseVelocity:FMat, baseAngularV:FMat):Boolean = {
	val baseVelocity0 = fromFMatToVector3(baseVelocity);
	val baseAngularV0 = fromFMatToVector3(baseAngularV);
	javaBullet.resetBaseVelocity(bodyUniqueId, baseVelocity0, baseAngularV0);
    };

    def applyExternalForce(objectUniqueId:Int, linkIndex:Int, force:DMat, position:DMat, flags:Int) = {
	javaBullet.applyExternalForce(objectUniqueId, linkIndex, force.data, position.data, flags);
    };
	
    def applyExternalTorque(objectUniqueId:Int, linkIndex:Int, torque:DMat, flags:Int) = {
	javaBullet.applyExternalTorque(objectUniqueId, linkIndex, torque.data, flags);
    };

    def getNumBodies() = {
	javaBullet.getNumBodies;
    };

    def getBodyInfo(bodyUniqueId:Int):BodyInfo = {
	val bodyInfo = new BodyInfo();
	javaBullet.getBodyInfo(bodyUniqueId, bodyInfo.javaBodyInfo);
	bodyInfo;
    };

    def getBodyUniqueId(bodyId:Int) = {
	javaBullet.getBodyUniqueId(bodyId)
    };

    def removeBody(bodyUniqeId:Int) = {
	javaBullet.removeBody(bodyUniqeId);
    };

    def createConstraint(parentBodyIndex:Int, parentJointIndex:Int, childBodyIndex:Int, childJointIndex:Int, jointInfo:JointInfo):Int = {

	javaBullet.createConstraint(parentBodyIndex, parentJointIndex, childBodyIndex, childJointIndex, jointInfo.javaJointInfo);
	
    };

    def createConstraint(parentBodyIndex:Int, parentJointIndex:Int, childBodyIndex:Int, childJointIndex:Int,
			 jointType:Int, jointAxis:DMat, parentFramePosition:DMat, childFramePosition:DMat,
			 parentFrameOrientation:DMat = null, childFramOrientation:DMat = null):Int = {
	
	javaBullet.createConstraint(parentBodyIndex, parentJointIndex, childBodyIndex, childJointIndex,
				    jointType, jointAxis.data, parentFramePosition.data, childFramePosition.data,
				    getData(parentFrameOrientation), getData(childFramOrientation));
    };

    def changeConstraint(constraintId:Int, jointInfo:JointInfo):Int = {
	javaBullet.changeConstraint(constraintId, jointInfo.javaJointInfo);
    };

    def removeConstraint(constraintId:Int):Unit = {
	javaBullet.removeConstraint(constraintId);
    };

    def getNumConstraints():Int = {
	javaBullet.getNumConstraints();
    };

    def getConstraintUniqueId(serialIndex:Int):Int = {
	javaBullet.getConstraintUniqueId(serialIndex);
    };

    def getDynamicsInfo(bodyUniqueId:Int, jointIndex:Int):DynamicsInfo = {
	val dynamicsInfo = new DynamicsInfo();
	javaBullet.getDynamicsInfo(bodyUniqueId, jointIndex, dynamicsInfo.javaDynamicsInfo);
	dynamicsInfo;
    };

    def changeDynamics(bodyUniqueId:Int, linkIndex:Int, mass:Double, lateralFriction:Double= -1, spinningFriction:Double= -1,
		       rollingFriction:Double= -1, restitution:Double= -1, linearDamping:Double= -1, angularDamping:Double= -1,
		       contactStiffness:Double= -1, contactDamping:Double= -1, frictionAnchor:Int= -1):Boolean = {
	
	javaBullet.changeDynamics(bodyUniqueId, linkIndex, mass, lateralFriction, spinningFriction,
				  rollingFriction, restitution, linearDamping, angularDamping,
				  contactStiffness, contactDamping, frictionAnchor);
    };

    def	setTimeStep(t:Double):Unit = {
	javaBullet.setTimeStep(t);
    };

    def setPhysicsEngineParameter(fixedTimeStep:Double= -1, numSolverIterations:Int= -1, useSplitImpulse:Int= -1,
				  splitImpulsePenetrationThreshold:Double= -1, numSubSteps:Int= -1,
				  collisionFilterMode:Int= -1, contactBreakingThreshold:Double= -1,
				  maxNumCmdPer1ms:Int= -1, enableFileCaching:Int= -1, restitutionVelocityThreshold:Double= -1,
				  erp:Double= -1, contactERP:Double= -1, frictionERP:Double= -1) = {
	
	javaBullet.setPhysicsEngineParameter(fixedTimeStep, numSolverIterations, useSplitImpulse,
					     splitImpulsePenetrationThreshold, numSubSteps,
					     collisionFilterMode, contactBreakingThreshold,
					     maxNumCmdPer1ms, enableFileCaching, restitutionVelocityThreshold,
					     erp, contactERP, frictionERP);
    };	

    def resetSimulation():Unit = {
	javaBullet.resetSimulation();
    };

    def startStateLogging(loggingType:Int, fileName:String, objectUniqueIds:IMat, maxLogDof:Int):Unit = {
	val objectUniqueIds0 = objectUniqueIds.data;
	javaBullet.startStateLogging(loggingType, fileName, objectUniqueIds0, maxLogDof);
    };

    def stopStateLogging(stateLoggerUniqueId:Int):Unit = { 
	javaBullet.stopStateLogging(stateLoggerUniqueId);
    };

    def computeViewMatrix(cameraPosition:FMat, cameraTargetPosition:FMat, cameraUp:FMat):FMat = {
	val viewMatrix = FMat.zeros(4,4);

	edu.berkeley.bid.Bullet.computeViewMatrix(cameraPosition.data, cameraTargetPosition.data,
						  cameraUp.data, viewMatrix.data);
	viewMatrix;
    };

    def computeViewMatrixFromYawPitchRoll(cameraTargetPosition:FMat, distance:Float, yaw:Float, pitch:Float, roll:Float, upAxisIndex:Int) = {
	val viewMatrix = FMat.zeros(4,4);

	edu.berkeley.bid.Bullet.computeViewMatrixFromYawPitchRoll(cameraTargetPosition.data, distance, yaw, pitch, roll, upAxisIndex, viewMatrix.data);
	viewMatrix;
    };

    def computeProjectionMatrix(left:Float, right:Float, bottom:Float, top:Float,
				nearVal:Float, farVal:Float):FMat = {
	val projectionMatrix = FMat.zeros(4,4);

	edu.berkeley.bid.Bullet.computeProjectionMatrix(left, right, bottom, top, nearVal, farVal, projectionMatrix.data);

	projectionMatrix;
    };

    def computeProjectionMatrixFOV(fov:Float, aspect:Float, nearVal:Float, farVal:Float):FMat = {
	val projectionMatrix = FMat.zeros(4,4);

	edu.berkeley.bid.Bullet.computeProjectionMatrixFOV(fov, aspect, nearVal, farVal, projectionMatrix.data);

	projectionMatrix;
    };

    def getCameraImage(width:Int, height:Int,
		       viewMatrix:FMat=null, projectionMatrix:FMat=null,
		       lightProjection:FMat=null, lightColor:FMat=null,
		       lightDistance:Float= -1f, hasShadow:Int = -1,
		       lightAmbientCoeff:Float = -1f, lightDiffuseCoeff:Float = -1f, lightSpecularCoeff:Float = -1f,
		       renderer:Int = -1):CameraImageData = {

	val cameraImage = new CameraImageData(width, height);

	javaBullet.getCameraImage(width, height,
				  fromFMatToFloatArray(viewMatrix), fromFMatToFloatArray(projectionMatrix),
				  fromFMatToFloatArray(lightProjection), fromFMatToFloatArray(lightColor),
				  lightDistance, hasShadow,
				  lightAmbientCoeff, lightDiffuseCoeff, lightSpecularCoeff,
				  renderer, cameraImage.javaCameraImageData);

	cameraImage;
    };

    def getCameraImageInts1(width:Int, height:Int,
				viewMatrix:FMat=null, projectionMatrix:FMat=null,
				lightProjection:FMat=null, lightColor:FMat=null,
				lightDistance:Float= -1f, hasShadow:Int = -1,
				lightAmbientCoeff:Float = -1f, lightDiffuseCoeff:Float = -1f, lightSpecularCoeff:Float = -1f,
				renderer:Int = -1):IMat = {

	val cameraImage = IMat.izeros(width, height);
	
	javaBullet.getCameraImageInts(width, height,
				      fromFMatToFloatArray(viewMatrix), fromFMatToFloatArray(projectionMatrix),
				      fromFMatToFloatArray(lightProjection), fromFMatToFloatArray(lightColor),
				      lightDistance, hasShadow,
				      lightAmbientCoeff, lightDiffuseCoeff, lightSpecularCoeff,
				      renderer, cameraImage.data, null, null);
	
	cameraImage;
    };

    def getCameraImageInts3(width:Int, height:Int,
			    viewMatrix:FMat=null, projectionMatrix:FMat=null,
			    lightProjection:FMat=null, lightColor:FMat=null,
			    lightDistance:Float= -1f, hasShadow:Int = -1,
			    lightAmbientCoeff:Float = -1f, lightDiffuseCoeff:Float = -1f, lightSpecularCoeff:Float = -1f,
			    renderer:Int = -1):(IMat, FMat, IMat) = {

	val cameraImage = IMat.izeros(width, height);
	val depthImage = FMat.zeros(width, height);
	val segmentation = IMat.izeros(width, height);
	
	javaBullet.getCameraImageInts(width, height,
				      fromFMatToFloatArray(viewMatrix), fromFMatToFloatArray(projectionMatrix),
				      fromFMatToFloatArray(lightProjection), fromFMatToFloatArray(lightColor),
				      lightDistance, hasShadow,
				      lightAmbientCoeff, lightDiffuseCoeff, lightSpecularCoeff,
				      renderer, cameraImage.data, depthImage.data, segmentation.data);
	
	(cameraImage, depthImage, segmentation)
    };

    def getCameraImageBytes1(width:Int, height:Int,
			     viewMatrix:FMat=null, projectionMatrix:FMat=null,
			     lightProjection:FMat=null, lightColor:FMat=null,
			     lightDistance:Float= -1f, hasShadow:Int = -1,
			     lightAmbientCoeff:Float = -1f, lightDiffuseCoeff:Float = -1f, lightSpecularCoeff:Float = -1f,
			     renderer:Int = -1):BMat = {

	val cameraImage = BMat.bzeros(MatFunctions.irow(4,height,width));
	
	javaBullet.getCameraImageBytes(width, height,
				       fromFMatToFloatArray(viewMatrix), fromFMatToFloatArray(projectionMatrix),
				       fromFMatToFloatArray(lightProjection), fromFMatToFloatArray(lightColor),
				       lightDistance, hasShadow,
				       lightAmbientCoeff, lightDiffuseCoeff, lightSpecularCoeff,
				       renderer, cameraImage.data, null, null);

	cameraImage;
    };

    def getCameraImageBytes3(width:Int, height:Int,
			     viewMatrix:FMat=null, projectionMatrix:FMat=null,
			     lightProjection:FMat=null, lightColor:FMat=null,
			     lightDistance:Float= -1f, hasShadow:Int = -1,
			     lightAmbientCoeff:Float = -1f, lightDiffuseCoeff:Float = -1f, lightSpecularCoeff:Float = -1f,
			     renderer:Int = -1):(BMat,FMat,IMat) = {

	val cameraImage = BMat.bzeros(MatFunctions.irow(4,height,width));
	val depthImage = FMat.zeros(width, height);
	val segmentation = IMat.izeros(width, height);
	
	javaBullet.getCameraImageBytes(width, height,
				       fromFMatToFloatArray(viewMatrix), fromFMatToFloatArray(projectionMatrix),
				       fromFMatToFloatArray(lightProjection), fromFMatToFloatArray(lightColor),
				       lightDistance, hasShadow,
				       lightAmbientCoeff, lightDiffuseCoeff, lightSpecularCoeff,
				       renderer, cameraImage.data, depthImage.data, segmentation.data);

	(cameraImage, depthImage, segmentation);
    };

    def getAABB(bodyUniqueId:Int, linkIndex:Int):(DMat, DMat) = {
	val AABBMin = DMat.zeros(1,3);
	val AABBMax = DMat.zeros(1,3);
	
	javaBullet.getAABB(bodyUniqueId, linkIndex, AABBMin.data, AABBMax.data);
	
	(AABBMin, AABBMax);
    };

    def getOverlappingObjects(AABBMin:DMat, AABBMax:DMat) = {
	javaBullet.getOverlappingObjects(AABBMin.data, AABBMax.data);
    };

    def getContactPoints(bodyUniqueIdA:Int, bodyUniqueIdB:Int, linkIndexA:Int, linkIndexB:Int) = {
	javaBullet.getContactPoints(bodyUniqueIdA, bodyUniqueIdB, linkIndexA, linkIndexB);
    };

    def getClosestPoints(bodyUniqueIdA:Int, bodyUniqueIdB:Int, distance:Double, linkIndexA:Int, linkIndexB:Int) = {
	javaBullet.getClosestPoints(bodyUniqueIdA, bodyUniqueIdB, distance, linkIndexA, linkIndexB);
    };

    def calculateInverseDynamics(bodyUniqueId:Int, jointPositions:DMat, jointVelocities:DMat, jointAccelerations:DMat):DMat = {
	val numJoints = getNumJoints(bodyUniqueId);
	val jointForcesOutput = DMat(1, numJoints);
	javaBullet.calculateInverseDynamics(bodyUniqueId, jointPositions.data, jointVelocities.data, jointAccelerations.data, jointForcesOutput.data);
	jointForcesOutput;
    };

    def calculateJacobian(bodyUniqueId:Int, linkIndex:Int, localPosition:DMat, jointPositions:DMat, jointVelocities:DMat, jointAccelerations:DMat):(DMat, DMat) = {
	val numJoints = getNumJoints(bodyUniqueId);
	val linearJacobian = DMat(numJoints, 3);
	val angularJacobian = DMat(numJoints, 3);
	javaBullet.getBodyJacobian(bodyUniqueId, linkIndex, localPosition.data, jointPositions.data, jointVelocities.data, jointAccelerations.data,
				   linearJacobian.data, angularJacobian.data);
	(linearJacobian.t, angularJacobian.t);
    };

    def calculateInverseKinematics(bodyUniqueId:Int, endEffectorLinkIndex:Int,
				   endEffectorTargetPosition:DMat, endEffectorTargetOrientation:DMat=null, 
				   lowerLimits:DMat=null, upperLimits:DMat=null, jointRanges:DMat=null, restPoses:DMat=null,
				   jointDamping:DMat=null):DMat = {
	val numJoints = getNumJoints(bodyUniqueId);
	val jointAnglesOutput = DMat(1, numJoints);
	javaBullet.calculateInverseKinematics(bodyUniqueId, endEffectorLinkIndex,
					      endEffectorTargetPosition.data, getData(endEffectorTargetOrientation), 
					      getData(lowerLimits), getData(upperLimits), getData(jointRanges), getData(restPoses),
					      getData(jointDamping), jointAnglesOutput.data);
	jointAnglesOutput;
    };

    def addUserDebugLine(fromXYZ:DMat, toXYZ:DMat, colorRGB:DMat= MatFunctions.drow(1,1,1), lineWidth:Double=1, lifeTime:Double= 0,
			 parentObjectUniqueId:Int= -1, parentLinkIndex:Int= -1) = {
	javaBullet.addUserDebugLine(fromXYZ.data, toXYZ.data, colorRGB.data, lineWidth, lifeTime, parentObjectUniqueId, parentLinkIndex);
    };

    def addUserDebugText3D(text:String, position:DMat, orientation:DMat, colorRGB:DMat= MatFunctions.drow(1,1,1), size:Double= 1, lifeTime:Double= 0,
			   parentObjectUniqueId:Int= -1, parentLinkIndex:Int= -1) = {
	javaBullet.addUserDebugText3D(text, position.data, getData(orientation), colorRGB.data, size, lifeTime, parentObjectUniqueId, parentLinkIndex);
    };

    def addUserDebugParameter(paramName:String, rangeMin:Double, rangeMax:Double, startValue:Double):Int = {
	javaBullet.addUserDebugParameter(paramName, rangeMin, rangeMax, startValue);
    };

    def readUserDebugParameter(itemUniqueId:Int):Double = {
	javaBullet.readUserDebugParameter(itemUniqueId);
    };

    def removeUserDebugItem(itemUniqueId:Int):Boolean = {
	javaBullet.removeUserDebugItem(itemUniqueId);
    };

    def configureDebugVisualizer(flags:Int, enable:Int):Unit = {
	javaBullet.configureDebugVisualizer(flags, enable);
    };

    def getDebugVisualizerCamera() = {
	javaBullet.getDebugVisualizerCamera();
    };

    def resetDebugVisualizerCamera(cameraDistance:Double, cameraPitch:Double, cameraYaw:Double, targetPos:FMat):Unit = {
	val targetPos0 = fromFMatToVector3(targetPos);
	javaBullet.resetDebugVisualizerCamera(cameraDistance, cameraPitch, cameraYaw, targetPos0);
    };
    
    def getKeyboardEventsData():KeyboardEventsData = {
	val keyboardEventsData = new KeyboardEventsData();
	javaBullet.getKeyboardEventsData(keyboardEventsData);
	keyboardEventsData;
    };

    def getMouseEventsData():MouseEventsData = {
	val mouseEventsData = new MouseEventsData();
	javaBullet.getMouseEventsData(mouseEventsData);
	mouseEventsData;
    };

    def setInternalSimFlags(flags:Int):Unit = {
	javaBullet.setInternalSimFlags(flags);
    };
	
    def setNumSimulationSubSteps(numSubSteps:Int):Unit = {
	javaBullet.setNumSimulationSubSteps(numSubSteps);
    };

    def setNumSolverIterations(numSolverIterations:Int):Unit = {
	javaBullet.setNumSolverIterations(numSolverIterations);
    };

    def setContactBreakingThreshold(threshold:Double):Unit = {
	javaBullet.setContactBreakingThreshold(threshold);
    };

    def fromFMatToFloatArray(mat:FMat):Array[Float] = {
	if (mat.asInstanceOf[AnyRef] == null) {
	    null;
	} else {
	    mat.data;
	}
    };

    def renderScene():Unit = {
	javaBullet.renderScene();
    };

    def debugDraw():Unit = {
	javaBullet.debugDraw();
    };

    def setTimeOut(t:Double):Unit = {
	javaBullet.setTimeOut(t);
    };

    def syncBodies():Unit = {
	javaBullet.syncBodies();
    };

    def canSubmitCommand():Boolean = {
	javaBullet.canSubmitCommand();
    };

    def submitProfileTiming(profileName:String, durationInMicroseconds:Int):Unit = {
	javaBullet.submitProfileTiming(profileName, durationInMicroseconds);
    };

    def getData(a:FMat):Array[Float]= {
	if (a.asInstanceOf[AnyRef] != null) {
	    a.data;
	} else {
	    null;
	}
    };

    def getData(a:DMat):Array[Double]= {
	if (a.asInstanceOf[AnyRef] != null) {
	    a.data;
	} else {
	    null;
	}
    }


}

object Bullet {

    def BIDMatQtoJavaQ(q:BIDMat.Quaternion):edu.berkeley.bid.bullet.Quaternion = {
	if (q.asInstanceOf[AnyRef] != null) {
	    new edu.berkeley.bid.bullet.Quaternion(q.data(1),q.data(2),q.data(3),q.data(0));
	} else {
	    null;
	}
    }

    def JavaQtoBIDMatQ(q:edu.berkeley.bid.bullet.Quaternion):BIDMat.Quaternion = {
	if (q.asInstanceOf[AnyRef] != null) {
	    BIDMat.Quaternion(q.w, q.x, q.y, q.z);
	} else {
	    null;
	}
    }

    def fromVector3ToFMat(v:Vector3):FMat = {
	if (v.asInstanceOf[AnyRef] != null) {
	    MatFunctions.row(v.x, v.y, v.z);
	} else {
	    null;
	}
    }

    def fromVector3ToDMat(v:Vector3):DMat = {
	if (v.asInstanceOf[AnyRef] != null) {
	    MatFunctions.drow(v.x, v.y, v.z);
	} else {
	    null;
	}
    }

    def fromFMatToVector3(v:FMat):Vector3 = {
	if (v.asInstanceOf[AnyRef] != null) {
	    new Vector3(v.data(0), v.data(1), v.data(2));
	} else {
	    null;
	}
    }

    def fromDMatToVector3(v:DMat):Vector3 = {
	if (v.asInstanceOf[AnyRef] != null) {
	    new Vector3(v.data(0).toFloat, v.data(1).toFloat, v.data(2).toFloat);
	} else {
	    null;
	}
    }




    final val eCONNECT_GUI = 1;
    final val eCONNECT_DIRECT = 2;
    final val eCONNECT_SHARED_MEMORY = 3;
    final val eCONNECT_UDP = 4;
    final val eCONNECT_TCP = 5;
    final val eCONNECT_EXISTING_EXAMPLE_BROWSER=6;
    final val eCONNECT_GUI_SERVER=7;

    final val COV_ENABLE_GUI=1;
    final val COV_ENABLE_SHADOWS=2;
    final val COV_ENABLE_WIREFRAME=3;
    final val COV_ENABLE_VR_TELEPORTING=4;
    final val COV_ENABLE_VR_PICKING=5;
    final val COV_ENABLE_VR_RENDER_CONTROLLERS=6;
    final val COV_ENABLE_RENDERING=7;
    final val COV_ENABLE_SYNC_RENDERING_INTERNAL=8;
    final val COV_ENABLE_KEYBOARD_SHORTCUTS=9;
    final val COV_ENABLE_MOUSE_PICKING=10;
    final val COV_ENABLE_Y_AXIS_UP=11;
    final val COV_ENABLE_TINY_RENDERER=12;
    final val COV_ENABLE_RGB_BUFFER_PREVIEW=13;
    final val COV_ENABLE_DEPTH_BUFFER_PREVIEW=14;
    final val COV_ENABLE_SEGMENTATION_MARK_PREVIEW=15;

    final val STATE_LOGGING_MINITAUR = 0;
    final val STATE_LOGGING_GENERIC_ROBOT = 1;
    final val STATE_LOGGING_VR_CONTROLLERS = 2;
    final val STATE_LOGGING_VIDEO_MP4 = 3;
    final val STATE_LOGGING_COMMANDS = 4;
    final val STATE_LOGGING_CONTACT_POINTS = 5;
    final val STATE_LOGGING_PROFILE_TIMINGS = 6;

    final val CONTROL_MODE_VELOCITY=0;
    final val CONTROL_MODE_TORQUE=1;
    final val CONTROL_MODE_POSITION_VELOCITY_PD=2;

    final val IK_DLS=0;
    final val IK_SDLS=1;
    final val IK_HAS_TARGET_POSITION=16;
    final val IK_HAS_TARGET_ORIENTATION=32;
    final val IK_HAS_NULL_SPACE_VELOCITY=64;
    final val IK_HAS_JOINT_DAMPING=128;

    final val EF_LINK_FRAME=1;
    final val EF_WORLD_FRAME=2;

    final val DEB_DEBUG_TEXT_ALWAYS_FACE_CAMERA=1;
    final val DEB_DEBUG_TEXT_USE_TRUE_TYPE_FONTS=2;
    final val DEB_DEBUG_TEXT_HAS_TRACKING_OBJECT=4;

    final val URDF_USE_INERTIA_FROM_FILE=2;
    final val URDF_USE_SELF_COLLISION=8;
    final val URDF_USE_SELF_COLLISION_EXCLUDE_PARENT=16;
    final val URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS=32;
    final val URDF_RESERVED=64;

    final val GEOM_SPHERE=2;
    final val GEOM_BOX=3;
    final val GEOM_CYLINDER=4;
    final val GEOM_MESH=5;
    final val GEOM_PLANE=6;
    final val GEOM_CAPSULE=7;
    final val GEOM_UNKNOWN=8; 

    final val GEOM_FORCE_CONCAVE_TRIMESH=1;
    
    final val GEOM_VISUAL_HAS_RGBA_COLOR=1;
    final val GEOM_VISUAL_HAS_SPECULAR_COLOR=2;

    final val STATE_LOG_JOINT_MOTOR_TORQUES=1;
    final val STATE_LOG_JOINT_USER_TORQUES=2;
    final val STATE_LOG_JOINT_TORQUES = STATE_LOG_JOINT_MOTOR_TORQUES+STATE_LOG_JOINT_USER_TORQUES;

    final val TORQUE_CONTROL=CONTROL_MODE_TORQUE;
    final val VELOCITY_CONTROL=CONTROL_MODE_VELOCITY;
    final val POSITION_CONTROL=CONTROL_MODE_POSITION_VELOCITY_PD;

}

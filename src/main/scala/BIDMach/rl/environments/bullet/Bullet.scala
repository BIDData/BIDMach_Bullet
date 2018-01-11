package BIDMach.rl.environments.bullet;
import edu.berkeley.bid.bullet.{Vector3,Matrix3x3}
import BIDMat.{BMat,DMat,FMat,IMat,Quaternion};
import BIDMat.MatFunctions._;


class Bullet {

    val javaBullet = new edu.berkeley.bid.bullet.Bullet();

    import BIDMach.rl.environments.bullet.Bullet._;

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
	irow(ints);
    };

    def loadMJCF(fname:String):IMat = {
	val ints:Array[Int] = javaBullet.loadMJCF(appendPathPrefix(fname));
	irow(ints);
    };

    def loadBullet(fname:String):IMat = {
	val ints:Array[Int] = javaBullet.loadBullet(appendPathPrefix(fname));
	irow(ints);
    };

    def createCollisionShape(shapeType:Int, radius:Double=0.5, halfExtents:DMat=drow(1,1,1), height:Double=1,
			     fileName:String=null, meshScale:DMat=drow(1,1,1), planeNormal:DMat=drow(0,0,1), flags:Int=0) = {
	
	javaBullet.createCollisionShape(shapeType, radius, getData(halfExtents), height,
					fileName, getData(meshScale), getData(planeNormal), flags);
    }

    def createMultiBody(baseMass:Double= -1, baseCollisionShapeIndex:Int = -1, baseVisualShapeIndex:Int = -1,
			basePosition:FMat = null, baseOrientation:Quaternion = null,
			baseInertialFramePosition:FMat = null, baseInertialFrameOrientation:Quaternion = null,

			linkMasses:FMat = null, linkCollisionShapeIndices:IMat = null, linkVisualShapeIndices:IMat = null,
			linkPositions:FMat = null, linkOrientations:FMat = null, 
			linkInertialFramePositions:FMat = null, linkInertialFrameOrientations:FMat = null,
			linkParentIndices:IMat = null, linkJointTypes:IMat = null, linkJointAxes:FMat = null, useMaximalCoordinates:Int = 0) = {

	javaBullet.createMultiBody(baseMass, baseCollisionShapeIndex, baseVisualShapeIndex,
				   fromFMatToVector3(basePosition), BIDMatQtoJavaQ(baseOrientation),
				   fromFMatToVector3(baseInertialFramePosition), BIDMatQtoJavaQ(baseInertialFrameOrientation),

				   getDataF2D(linkMasses), getData(linkCollisionShapeIndices), getData(linkVisualShapeIndices),
				   fromFMatToArrayVector3(linkPositions), fromFMatToArrayQuaternion(linkPositions),
				   fromFMatToArrayVector3(linkInertialFramePositions), fromFMatToArrayQuaternion(linkInertialFramePositions), 
				   getData(linkParentIndices), getData(linkJointTypes), fromFMatToArrayVector3(linkJointAxes),
				   useMaximalCoordinates);
    }


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

    def getQuaternionFromEuler(euler:FMat):BIDMat.Quaternion =
	Bullet.getQuaternionFromEuler(euler);

    def getQuaternionFromEuler(yawZ:Double, pitchY:Double, rollX:Double):BIDMat.Quaternion =
	Bullet.getQuaternionFromEuler(yawZ, pitchY, rollX);

    def getEulerFromQuaternion(q:BIDMat.Quaternion):FMat =
	Bullet.getEulerFromQuaternion(q);

    def getMatrixFromQuaternion(q:BIDMat.Quaternion):FMat = {
	q.toRotMat;
    };

    def getNumJoints(bodyUniqueId:Int):Int = {
	javaBullet.getNumJoints(bodyUniqueId);
    };

    def getJointInfo(bodyUniqueId:Int, jointIndex:Int):JointInfo = {
	val jointInfo = new JointInfo();
	javaBullet.getJointInfo(bodyUniqueId, jointIndex, jointInfo.javaJointInfo);
	jointInfo;
    };

    def setJointMotorControl(bodyIndex:Int, jointIndex:Int, controlMode:Int,
			     targetPosition:Double = 0, targetVelocity:Double = 0,
			     force:Double = 100000, positionGain:Double = 0.1, velocityGain:Double = 1.0):Unit = {
	javaBullet.setJointMotorControl(bodyIndex, jointIndex, controlMode, targetPosition, targetVelocity, force, positionGain, velocityGain);
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

    def enableJointForceTorqueSensor(bodyUniqueId:Int, jointIndex:Int, enable:Boolean=true) = {
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

    def applyExternalForce(objectUniqueId:Int, linkIndex:Int, force:FMat, position:FMat, flags:Int) = {
	javaBullet.applyExternalForce(objectUniqueId, linkIndex, getDataF2D(force), getDataF2D(position), flags);
    };
	
    def applyExternalTorque(objectUniqueId:Int, linkIndex:Int, torque:FMat, flags:Int) = {
	javaBullet.applyExternalTorque(objectUniqueId, linkIndex, getDataF2D(torque), flags);
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
			 jointType:Int, jointAxis:FMat, parentFramePosition:FMat, childFramePosition:FMat,
			 parentFrameOrientation:Quaternion = null, childFramOrientation:Quaternion = null):Int = {
	
	javaBullet.createConstraint(parentBodyIndex, parentJointIndex, childBodyIndex, childJointIndex,
				    jointType, jointAxis.data, parentFramePosition.data, childFramePosition.data,
				    getData(parentFrameOrientation), getData(childFramOrientation));
    };

    def changeConstraint(constraintId:Int, jointChildPivot:FMat = null, jointChildFrameOrientation:Quaternion = null, maxForce:Double = -1) = {
	javaBullet.changeConstraint(constraintId, getData(jointChildPivot), getData(jointChildFrameOrientation), maxForce);
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

    def changeDynamics(bodyUniqueId:Int, linkIndex:Int, mass:Double= -1, lateralFriction:Double= -1, spinningFriction:Double= -1,
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

    def startStateLogging(loggingType:Int, fileName:String, objectUniqueIds:IMat=null, maxLogDof:Int= -1):Int = {
	val objectUniqueIds0 = if (objectUniqueIds.asInstanceOf[AnyRef] != null) objectUniqueIds.data else null;
	javaBullet.startStateLogging(loggingType, fileName, objectUniqueIds0, maxLogDof);
    };

    def stopStateLogging(stateLoggerUniqueId:Int):Unit = { 
	javaBullet.stopStateLogging(stateLoggerUniqueId);
    };

    def computeViewMatrix(cameraPosition:FMat, cameraTargetPosition:FMat, cameraUp:FMat):FMat =
	Bullet.computeViewMatrix(cameraPosition, cameraTargetPosition, cameraUp);

    def computeViewMatrixFromYawPitchRoll(cameraTargetPosition:FMat, distance:Float, yaw:Float, pitch:Float, roll:Float, upAxisIndex:Int) =
	Bullet.computeViewMatrixFromYawPitchRoll(cameraTargetPosition, distance, yaw, pitch, roll, upAxisIndex);

    def computeProjectionMatrix(left:Float, right:Float, bottom:Float, top:Float, nearVal:Float, farVal:Float):FMat =
	Bullet.computeProjectionMatrix(left, right, bottom, top, nearVal, farVal);

    def computeProjectionMatrixFOV(fov:Float, aspect:Float, nearVal:Float, farVal:Float):FMat =
	Bullet.computeProjectionMatrixFOV(fov, aspect, nearVal, farVal);

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

	val cameraImage = BMat.bzeros(irow(4,height,width));
	
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

	val cameraImage = BMat.bzeros(irow(4,height,width));
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

    def getAABB(bodyUniqueId:Int, linkIndex:Int = -1):(FMat, FMat) = {
	val AABBMin = DMat.zeros(1,3);
	val AABBMax = DMat.zeros(1,3);
	
	javaBullet.getAABB(bodyUniqueId, linkIndex, AABBMin.data, AABBMax.data);
	
	(FMat(AABBMin), FMat(AABBMax));
    };

    def getOverlappingObjects(AABBMin:FMat, AABBMax:FMat) = {
	javaBullet.getOverlappingObjects(getDataF2D(AABBMin), getDataF2D(AABBMax));
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

    def addUserDebugLine(fromXYZ:FMat, toXYZ:FMat, textColorRGB:FMat= row(1,1,1), lineWidth:Double=1, lifeTime:Double= 0,
			 parentObjectUniqueId:Int= -1, parentLinkIndex:Int= -1) = {
	javaBullet.addUserDebugLine(getDataF2D(fromXYZ), getDataF2D(toXYZ), getDataF2D(textColorRGB), lineWidth, lifeTime, parentObjectUniqueId, parentLinkIndex);
    };

    def addUserDebugText(text:String, position:FMat, orientation:Quaternion= null, textColorRGB:FMat= row(1,1,1), textSize:Double= 1, lifeTime:Double= 0,
			   parentObjectUniqueId:Int= -1, parentLinkIndex:Int= -1) = {
	javaBullet.addUserDebugText(text, getDataF2D(position), getDataF2D(orientation), getDataF2D(textColorRGB), textSize, lifeTime, parentObjectUniqueId, parentLinkIndex);
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
    };

    def getData(a:IMat):Array[Int]= {
	if (a.asInstanceOf[AnyRef] != null) {
	    a.data;
	} else {
	    null;
	}
    };

    def getDataF2D(a:FMat):Array[Double]= {
	if (a.asInstanceOf[AnyRef] != null) {
	    val b = new Array[Double](a.length);
	    var i = 0;
	    val len = a.length;
	    while (i < len) {
		b(i) = a.data(i);
		i += 1;
	    };
	    b;	
	} else {
	    null;
	}
    };
    


    final val SHARED_MEMORY = edu.berkeley.bid.bullet.Bullet.eCONNECT_SHARED_MEMORY;
    final val DIRECT = edu.berkeley.bid.bullet.Bullet.eCONNECT_DIRECT;
    final val GUI = edu.berkeley.bid.bullet.Bullet.eCONNECT_GUI;
    final val UDP = edu.berkeley.bid.bullet.Bullet.eCONNECT_UDP;
    final val TCP = edu.berkeley.bid.bullet.Bullet.eCONNECT_TCP;
    final val GUI_SERVER = edu.berkeley.bid.bullet.Bullet.eCONNECT_GUI_SERVER;

    final val JOINT_REVOLUTE = edu.berkeley.bid.bullet.Bullet.eRevoluteType;
    final val JOINT_PRISMATIC = edu.berkeley.bid.bullet.Bullet.ePrismaticType;
    final val JOINT_SPHERICAL = edu.berkeley.bid.bullet.Bullet.eSphericalType;
    final val JOINT_PLANAR = edu.berkeley.bid.bullet.Bullet.ePlanarType;
    final val JOINT_FIXED = edu.berkeley.bid.bullet.Bullet.eFixedType;
    final val JOINT_POINT2POINT = edu.berkeley.bid.bullet.Bullet.ePoint2PointType;
    final val JOINT_GEAR = edu.berkeley.bid.bullet.Bullet.eGearType;

    final val SENSOR_FORCE_TORQUE = edu.berkeley.bid.bullet.Bullet.eSensorForceTorqueType;

    final val TORQUE_CONTROL = edu.berkeley.bid.bullet.Bullet.CONTROL_MODE_TORQUE;
    final val VELOCITY_CONTROL = edu.berkeley.bid.bullet.Bullet.CONTROL_MODE_VELOCITY;
    final val POSITION_CONTROL = edu.berkeley.bid.bullet.Bullet.CONTROL_MODE_POSITION_VELOCITY_PD;

    final val LINK_FRAME = edu.berkeley.bid.bullet.Bullet.EF_LINK_FRAME;
    final val WORLD_FRAME = edu.berkeley.bid.bullet.Bullet.EF_WORLD_FRAME;

    final val CONTACT_REPORT_EXISTING = edu.berkeley.bid.bullet.Bullet.CONTACT_QUERY_MODE_REPORT_EXISTING_CONTACT_POINTS;
    final val CONTACT_RECOMPUTE_CLOSEST = edu.berkeley.bid.bullet.Bullet.CONTACT_QUERY_MODE_COMPUTE_CLOSEST_POINTS;

    final val VR_BUTTON_IS_DOWN = edu.berkeley.bid.bullet.Bullet.eButtonIsDown;
    final val VR_BUTTON_WAS_TRIGGERED = edu.berkeley.bid.bullet.Bullet.eButtonTriggered;
    final val VR_BUTTON_WAS_RELEASED = edu.berkeley.bid.bullet.Bullet.eButtonReleased;

    final val VR_MAX_CONTROLLERS = edu.berkeley.bid.bullet.Bullet.MAX_VR_CONTROLLERS;
    final val VR_MAX_BUTTONS = edu.berkeley.bid.bullet.Bullet.MAX_VR_BUTTONS;

    final val VR_DEVICE_CONTROLLER = edu.berkeley.bid.bullet.Bullet.VR_DEVICE_CONTROLLER;
    final val VR_DEVICE_HMD = edu.berkeley.bid.bullet.Bullet.VR_DEVICE_HMD;
    final val VR_DEVICE_GENERIC_TRACKER = edu.berkeley.bid.bullet.Bullet.VR_DEVICE_GENERIC_TRACKER;

    final val VR_CAMERA_TRACK_OBJECT_ORIENTATION = edu.berkeley.bid.bullet.Bullet.VR_CAMERA_TRACK_OBJECT_ORIENTATION;

    final val KEY_IS_DOWN = edu.berkeley.bid.bullet.Bullet.eButtonIsDown;
    final val KEY_WAS_TRIGGERED = edu.berkeley.bid.bullet.Bullet.eButtonTriggered;
    final val KEY_WAS_RELEASED = edu.berkeley.bid.bullet.Bullet.eButtonReleased;

    final val STATE_LOGGING_MINITAUR = edu.berkeley.bid.bullet.Bullet.STATE_LOGGING_MINITAUR;
    final val STATE_LOGGING_GENERIC_ROBOT = edu.berkeley.bid.bullet.Bullet.STATE_LOGGING_GENERIC_ROBOT;
    final val STATE_LOGGING_VR_CONTROLLERS = edu.berkeley.bid.bullet.Bullet.STATE_LOGGING_VR_CONTROLLERS;
    final val STATE_LOGGING_VIDEO_MP4 = edu.berkeley.bid.bullet.Bullet.STATE_LOGGING_VIDEO_MP4;
    final val STATE_LOGGING_CONTACT_POINTS = edu.berkeley.bid.bullet.Bullet.STATE_LOGGING_CONTACT_POINTS;
    final val STATE_LOGGING_PROFILE_TIMINGS = edu.berkeley.bid.bullet.Bullet.STATE_LOGGING_PROFILE_TIMINGS;

    final val COV_ENABLE_GUI = edu.berkeley.bid.bullet.Bullet.COV_ENABLE_GUI;
    final val COV_ENABLE_SHADOWS = edu.berkeley.bid.bullet.Bullet.COV_ENABLE_SHADOWS;
    final val COV_ENABLE_WIREFRAME = edu.berkeley.bid.bullet.Bullet.COV_ENABLE_WIREFRAME;
    final val COV_ENABLE_VR_PICKING = edu.berkeley.bid.bullet.Bullet.COV_ENABLE_VR_PICKING;
    final val COV_ENABLE_VR_TELEPORTING = edu.berkeley.bid.bullet.Bullet.COV_ENABLE_VR_TELEPORTING;
    final val COV_ENABLE_RENDERING = edu.berkeley.bid.bullet.Bullet.COV_ENABLE_RENDERING;
    final val COV_ENABLE_VR_RENDER_CONTROLLERS = edu.berkeley.bid.bullet.Bullet.COV_ENABLE_VR_RENDER_CONTROLLERS;
    final val COV_ENABLE_KEYBOARD_SHORTCUTS = edu.berkeley.bid.bullet.Bullet.COV_ENABLE_KEYBOARD_SHORTCUTS;
    final val COV_ENABLE_MOUSE_PICKING = edu.berkeley.bid.bullet.Bullet.COV_ENABLE_MOUSE_PICKING;


    final val ER_TINY_RENDERER = edu.berkeley.bid.bullet.Bullet.ER_TINY_RENDERER;
    final val ER_BULLET_HARDWARE_OPENGL = edu.berkeley.bid.bullet.Bullet.ER_BULLET_HARDWARE_OPENGL;

    final val URDF_USE_INERTIA_FROM_FILE = edu.berkeley.bid.bullet.Bullet.URDF_USE_INERTIA_FROM_FILE;
    final val URDF_USE_SELF_COLLISION = edu.berkeley.bid.bullet.Bullet.URDF_USE_SELF_COLLISION;
    final val URDF_USE_SELF_COLLISION_EXCLUDE_PARENT = edu.berkeley.bid.bullet.Bullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT;
    final val URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS = edu.berkeley.bid.bullet.Bullet.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS;

    final val MAX_RAY_INTERSECTION_BATCH_SIZE = edu.berkeley.bid.bullet.Bullet.MAX_RAY_INTERSECTION_BATCH_SIZE;

    final val B3G_F1 = edu.berkeley.bid.bullet.Bullet.B3G_F1;
    final val B3G_F2 = edu.berkeley.bid.bullet.Bullet.B3G_F2;
    final val B3G_F3 = edu.berkeley.bid.bullet.Bullet.B3G_F3;
    final val B3G_F4 = edu.berkeley.bid.bullet.Bullet.B3G_F4;
    final val B3G_F5 = edu.berkeley.bid.bullet.Bullet.B3G_F5;
    final val B3G_F6 = edu.berkeley.bid.bullet.Bullet.B3G_F6;
    final val B3G_F7 = edu.berkeley.bid.bullet.Bullet.B3G_F7;
    final val B3G_F8 = edu.berkeley.bid.bullet.Bullet.B3G_F8;
    final val B3G_F9 = edu.berkeley.bid.bullet.Bullet.B3G_F9;
    final val B3G_F10 = edu.berkeley.bid.bullet.Bullet.B3G_F10;
    final val B3G_F11 = edu.berkeley.bid.bullet.Bullet.B3G_F11;
    final val B3G_F12 = edu.berkeley.bid.bullet.Bullet.B3G_F12;
    final val B3G_F13 = edu.berkeley.bid.bullet.Bullet.B3G_F13;
    final val B3G_F14 = edu.berkeley.bid.bullet.Bullet.B3G_F14;
    final val B3G_F15 = edu.berkeley.bid.bullet.Bullet.B3G_F15;
    final val B3G_LEFT_ARROW = edu.berkeley.bid.bullet.Bullet.B3G_LEFT_ARROW;
    final val B3G_RIGHT_ARROW = edu.berkeley.bid.bullet.Bullet.B3G_RIGHT_ARROW;
    final val B3G_UP_ARROW = edu.berkeley.bid.bullet.Bullet.B3G_UP_ARROW;
    final val B3G_DOWN_ARROW = edu.berkeley.bid.bullet.Bullet.B3G_DOWN_ARROW;
    final val B3G_PAGE_UP = edu.berkeley.bid.bullet.Bullet.B3G_PAGE_UP;
    final val B3G_PAGE_DOWN = edu.berkeley.bid.bullet.Bullet.B3G_PAGE_DOWN;
    final val B3G_END = edu.berkeley.bid.bullet.Bullet.B3G_END;
    final val B3G_HOME = edu.berkeley.bid.bullet.Bullet.B3G_HOME;
    final val B3G_INSERT = edu.berkeley.bid.bullet.Bullet.B3G_INSERT;
    final val B3G_DELETE = edu.berkeley.bid.bullet.Bullet.B3G_DELETE;
    final val B3G_BACKSPACE = edu.berkeley.bid.bullet.Bullet.B3G_BACKSPACE;
    final val B3G_SHIFT = edu.berkeley.bid.bullet.Bullet.B3G_SHIFT;
    final val B3G_CONTROL = edu.berkeley.bid.bullet.Bullet.B3G_CONTROL;
    final val B3G_ALT = edu.berkeley.bid.bullet.Bullet.B3G_ALT;
    final val B3G_RETURN = edu.berkeley.bid.bullet.Bullet.B3G_RETURN;

    final val GEOM_SPHERE = edu.berkeley.bid.bullet.Bullet.GEOM_SPHERE;
    final val GEOM_BOX = edu.berkeley.bid.bullet.Bullet.GEOM_BOX;
    final val GEOM_CYLINDER = edu.berkeley.bid.bullet.Bullet.GEOM_CYLINDER;
    final val GEOM_MESH = edu.berkeley.bid.bullet.Bullet.GEOM_MESH;
    final val GEOM_PLANE = edu.berkeley.bid.bullet.Bullet.GEOM_PLANE;
    final val GEOM_CAPSULE = edu.berkeley.bid.bullet.Bullet.GEOM_CAPSULE;

    final val GEOM_FORCE_CONCAVE_TRIMESH = edu.berkeley.bid.bullet.Bullet.GEOM_FORCE_CONCAVE_TRIMESH;
	
    final val STATE_LOG_JOINT_MOTOR_TORQUES = edu.berkeley.bid.bullet.Bullet.STATE_LOG_JOINT_MOTOR_TORQUES;
    final val STATE_LOG_JOINT_USER_TORQUES = edu.berkeley.bid.bullet.Bullet.STATE_LOG_JOINT_USER_TORQUES;
    final val STATE_LOG_JOINT_TORQUES = edu.berkeley.bid.bullet.Bullet.STATE_LOG_JOINT_USER_TORQUES+STATE_LOG_JOINT_MOTOR_TORQUES;

}

object Bullet {

    def getQuaternionFromEuler(euler:FMat):BIDMat.Quaternion = {
	val euler0 = fromFMatToVector3(euler);
	val q = new edu.berkeley.bid.bullet.Quaternion();
	edu.berkeley.bid.bullet.Bullet.getQuaternionFromEuler(euler0, q);
	JavaQtoBIDMatQ(q);
    };

    def getQuaternionFromEuler(yawZ:Double, pitchY:Double, rollX:Double):BIDMat.Quaternion = {
	val euler0 = fromFMatToVector3(row(yawZ, pitchY, rollX));
	val q = new edu.berkeley.bid.bullet.Quaternion();
	edu.berkeley.bid.bullet.Bullet.getQuaternionFromEuler(euler0, q);
	JavaQtoBIDMatQ(q);
    };

    def getEulerFromQuaternion(q:BIDMat.Quaternion):FMat = {
	val v = new Vector3();
	val q0 = BIDMatQtoJavaQ(q);
	edu.berkeley.bid.bullet.Bullet.getEulerFromQuaternion(q0, v);
	fromVector3ToFMat(v);
    };

    def getMatrixFromQuaternion(q:BIDMat.Quaternion):FMat = {
	q.toRotMat;
    };

    def computeViewMatrix(cameraPosition:FMat, cameraTargetPosition:FMat, cameraUp:FMat):FMat = {
	val viewMatrix = FMat.zeros(4,4);

	edu.berkeley.bid.bullet.Bullet.computeViewMatrix(cameraPosition.data, cameraTargetPosition.data,
						  cameraUp.data, viewMatrix.data);
	viewMatrix;
    };

    def computeViewMatrixFromYawPitchRoll(cameraTargetPosition:FMat, distance:Float, yaw:Float, pitch:Float, roll:Float, upAxisIndex:Int) = {
	val viewMatrix = FMat.zeros(4,4);

	edu.berkeley.bid.bullet.Bullet.computeViewMatrixFromYawPitchRoll(cameraTargetPosition.data, distance, yaw, pitch, roll, upAxisIndex, viewMatrix.data);
	viewMatrix;
    };

    def computeProjectionMatrix(left:Float, right:Float, bottom:Float, top:Float,
				nearVal:Float, farVal:Float):FMat = {
	val projectionMatrix = FMat.zeros(4,4);

	edu.berkeley.bid.bullet.Bullet.computeProjectionMatrix(left, right, bottom, top, nearVal, farVal, projectionMatrix.data);

	projectionMatrix;
    };

    def computeProjectionMatrixFOV(fov:Float, aspect:Float, nearVal:Float, farVal:Float):FMat = {
	val projectionMatrix = FMat.zeros(4,4);

	edu.berkeley.bid.bullet.Bullet.computeProjectionMatrixFOV(fov, aspect, nearVal, farVal, projectionMatrix.data);

	projectionMatrix;
    };

    def BIDMatQtoJavaQ(q:BIDMat.Quaternion):edu.berkeley.bid.bullet.Quaternion = {
	if (q.asInstanceOf[AnyRef] != null) {
	    new edu.berkeley.bid.bullet.Quaternion(q.data(0),q.data(1),q.data(2),q.data(3));
	} else {
	    null;
	}
    }

    def JavaQtoBIDMatQ(q:edu.berkeley.bid.bullet.Quaternion):BIDMat.Quaternion = {
	if (q.asInstanceOf[AnyRef] != null) {
	    BIDMat.Quaternion(q.x, q.y, q.z, q.w);
	} else {
	    null;
	}
    }

    def fromVector3ToFMat(v:Vector3):FMat = {
	if (v.asInstanceOf[AnyRef] != null) {
	    row(v.x, v.y, v.z);
	} else {
	    null;
	}
    }

    def fromVector3ToDMat(v:Vector3):DMat = {
	if (v.asInstanceOf[AnyRef] != null) {
	    drow(v.x, v.y, v.z);
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

    def fromFMatToJavaQuaternion(v:FMat):edu.berkeley.bid.bullet.Quaternion = {
	if (v.asInstanceOf[AnyRef] != null) {
	    new edu.berkeley.bid.bullet.Quaternion(v.data(0), v.data(1), v.data(2), v.data(3));
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

    def fromFMatToArrayVector3(v:FMat):Array[Vector3] = {
	if (v.asInstanceOf[AnyRef] != null) {
	    val nvecs = v.ncols;
	    val out = new Array[Vector3](nvecs);
	    var i = 0;
	    while (i < nvecs) {
		out(i) = fromFMatToVector3(v(?,i));
		i += 1;
	    }
	    out;
	} else {
	    null;
	}
    }

    def fromFMatToArrayQuaternion(v:FMat):Array[edu.berkeley.bid.bullet.Quaternion] = {
	if (v.asInstanceOf[AnyRef] != null) {
	    val nvecs = v.ncols;
	    val out = new Array[edu.berkeley.bid.bullet.Quaternion](nvecs);
	    var i = 0;
	    while (i < nvecs) {
		out(i) = fromFMatToJavaQuaternion(v(?,i));
		i += 1;
	    }
	    out;
	} else {
	    null;
	}
    }

}

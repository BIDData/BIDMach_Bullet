package BIDMach.rl.environments.bullet;
import BIDMat.{FMat,DMat,IMat,Quaternion};
import BIDMat.MatFunctions._;
import scala.collection.mutable.HashMap;

class Minitaur(val p:Bullet, val urdfRootPath:String = "") {

    var quadruped:Int = 0;
    var kp:Double = 0;
    var kd:Double = 0;
    var maxForce:Double = 0;
    var nMotors:Int = 0;
    var motorIdList:IMat = null;
    var motorDir:DMat = null;
    var jointNameToId:HashMap[String,Int] = null;

    reset();

    def buildJointNameToIdDict() = {
	val nJoints = p.getNumJoints(quadruped);
	for (i <- 0 until nJoints) {
	    val jointInfo = p.getJointInfo(quadruped, i);
	    jointNameToId(jointInfo.jointName) = jointInfo.jointIndex;
	}
	resetPose();
	for (i <- 0 until 100) {
	    p.stepSimulation();
	}
    };

    def buildMotorIdList() = {
	motorIdList = irow(jointNameToId("motor_front_leftL_joint"),
			   jointNameToId("motor_front_leftR_joint"),
			   jointNameToId("motor_back_leftL_joint"),
			   jointNameToId("motor_back_leftR_joint"),
			   jointNameToId("motor_front_rightL_joint"),
			   jointNameToId("motor_front_rightR_joint"),
			   jointNameToId("motor_back_rightL_joint"),
			   jointNameToId("motor_back_rightR_joint"));
    };


    def reset() = {
	quadruped = p.loadURDF("%s/quadruped/minitaur.urdf" format urdfRootPath, row(0,0,0.2f));
	kp = 1.0;
	kd = 0.1;
	maxForce = 3.5;
	nMotors = 8;
	jointNameToId = new HashMap[String,Int]();
	motorDir = drow(-1, -1, -1, -1, 1, 1, 1, 1);

	buildJointNameToIdDict();
	buildMotorIdList();
    };

    def setMotorAngleById(motorId:Int, desiredAngle:Double) = {
	p.setJointMotorControl(bodyIndex=quadruped, jointIndex=motorId, controlMode=p.POSITION_CONTROL, targetPosition=desiredAngle, positionGain=kp, velocityGain=kd, force=maxForce);
    };

    def setMotorAngleByName(motorName:String, desiredAngle:Double) = {
	setMotorAngleById(jointNameToId(motorName), desiredAngle);
    };

    def resetPose() = {
	val kneeFrictionForce = 0;
	val halfpi = 1.57079632679;
	val kneeangle = -2.1834 // halfpi - acos(upper_leg_length / lower_leg_length)

	// left front leg
	p.resetJointState(quadruped,jointNameToId("motor_front_leftL_joint"),motorDir(0)*halfpi)
	p.resetJointState(quadruped,jointNameToId("knee_front_leftL_link"),motorDir(0)*kneeangle)
	p.resetJointState(quadruped,jointNameToId("motor_front_leftR_joint"),motorDir(1)*halfpi)
	p.resetJointState(quadruped,jointNameToId("knee_front_leftR_link"),motorDir(1)*kneeangle)
	p.createConstraint(quadruped,jointNameToId("knee_front_leftR_link"),quadruped,jointNameToId("knee_front_leftL_link"),p.JOINT_POINT2POINT,row(0,0,0),row(0,0.005f,0.2),row(0,0.01f,0.2f))
	setMotorAngleByName("motor_front_leftL_joint", motorDir(0)*halfpi)
	setMotorAngleByName("motor_front_leftR_joint", motorDir(1)*halfpi)
	p.setJointMotorControl(bodyIndex=quadruped,jointIndex=jointNameToId("knee_front_leftL_link"),controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
	p.setJointMotorControl(bodyIndex=quadruped,jointIndex=jointNameToId("knee_front_leftR_link"),controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)

	// left back leg
	p.resetJointState(quadruped,jointNameToId("motor_back_leftL_joint"),motorDir(2)*halfpi)
	p.resetJointState(quadruped,jointNameToId("knee_back_leftL_link"),motorDir(2)*kneeangle)
	p.resetJointState(quadruped,jointNameToId("motor_back_leftR_joint"),motorDir(3)*halfpi)
	p.resetJointState(quadruped,jointNameToId("knee_back_leftR_link"),motorDir(3)*kneeangle)
	p.createConstraint(quadruped,jointNameToId("knee_back_leftR_link"),quadruped,jointNameToId("knee_back_leftL_link"),p.JOINT_POINT2POINT,row(0,0,0),row(0,0.005f,0.2f),row(0,0.01f,0.2f))
	setMotorAngleByName("motor_back_leftL_joint",motorDir(2)*halfpi)
	setMotorAngleByName("motor_back_leftR_joint",motorDir(3)*halfpi)
	p.setJointMotorControl(bodyIndex=quadruped,jointIndex=jointNameToId("knee_back_leftL_link"),controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
	p.setJointMotorControl(bodyIndex=quadruped,jointIndex=jointNameToId("knee_back_leftR_link"),controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
 
	// right front leg
	p.resetJointState(quadruped,jointNameToId("motor_front_rightL_joint"),motorDir(4)*halfpi)
	p.resetJointState(quadruped,jointNameToId("knee_front_rightL_link"),motorDir(4)*kneeangle)
	p.resetJointState(quadruped,jointNameToId("motor_front_rightR_joint"),motorDir(5)*halfpi)
	p.resetJointState(quadruped,jointNameToId("knee_front_rightR_link"),motorDir(5)*kneeangle)
	p.createConstraint(quadruped,jointNameToId("knee_front_rightR_link"),quadruped,jointNameToId("knee_front_rightL_link"),p.JOINT_POINT2POINT,row(0,0,0),row(0,0.005f,0.2f),row(0,0.01f,0.2f))
	setMotorAngleByName("motor_front_rightL_joint",motorDir(4)*halfpi)
	setMotorAngleByName("motor_front_rightR_joint",motorDir(5)*halfpi)
	p.setJointMotorControl(bodyIndex=quadruped,jointIndex=jointNameToId("knee_front_rightL_link"),controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
	p.setJointMotorControl(bodyIndex=quadruped,jointIndex=jointNameToId("knee_front_rightR_link"),controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
 

	// right back leg
	p.resetJointState(quadruped,jointNameToId("motor_back_rightL_joint"),motorDir(6)*halfpi)
	p.resetJointState(quadruped,jointNameToId("knee_back_rightL_link"),motorDir(6)*kneeangle)
	p.resetJointState(quadruped,jointNameToId("motor_back_rightR_joint"),motorDir(7)*halfpi)
	p.resetJointState(quadruped,jointNameToId("knee_back_rightR_link"),motorDir(7)*kneeangle)
	p.createConstraint(quadruped,jointNameToId("knee_back_rightR_link"),quadruped,jointNameToId("knee_back_rightL_link"),p.JOINT_POINT2POINT,row(0,0,0),row(0,0.005f,0.2f),row(0,0.01f,0.2f))
	setMotorAngleByName("motor_back_rightL_joint",motorDir(6)*halfpi)
	setMotorAngleByName("motor_back_rightR_joint",motorDir(7)*halfpi)
	p.setJointMotorControl(bodyIndex=quadruped,jointIndex=jointNameToId("knee_back_rightL_link"),controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
	p.setJointMotorControl(bodyIndex=quadruped,jointIndex=jointNameToId("knee_back_rightR_link"),controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
    };


    def getBasePosition() = {
	val (position, orientation) = p.getBasePositionAndOrientation(quadruped);
	position;
    };
	
    def getBaseOrientation() = {
	val (position, orientation) = p.getBasePositionAndOrientation(quadruped);
	orientation;
    };

    def applyAction(motorCommands:DMat) = {
	val motorCommandsWithDir = motorCommands *@ motorDir;
	for (i <- 0 until nMotors) {
	    setMotorAngleById(motorIdList(i), motorCommandsWithDir(i));
	}
    };

    def getMotorAngles() = {
	val motorAngles = dzeros(1, nMotors);
	for (i <- 0 until nMotors) {
	    val jointState = p.getJointState(quadruped, motorIdList(i));
	    motorAngles(i) = jointState.position;
	}
	motorAngles *@ motorDir;
    };
	
    def getMotorVelocities() = {
	val motorVelocities = dzeros(1, nMotors);
	for (i <- 0 until nMotors) {
	    val jointState = p.getJointState(quadruped, motorIdList(i));
	    motorVelocities(i) = jointState.velocity;
	}
	motorVelocities *@ motorDir;
    };

    def getMotorTorques() = {
	val motorTorques = dzeros(1, nMotors);
	for (i <- 0 until nMotors) {
	    val jointState = p.getJointState(quadruped, motorIdList(i));
	    motorTorques(i) = jointState.motorTorque;
	}
	motorTorques *@ motorDir;
    }
}

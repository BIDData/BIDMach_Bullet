package BIDMach.rl.environments.bullet;
import BIDMat.{FMat,DMat,IMat,Quaternion};
import BIDMat.MatFunctions._;
import BIDMat.SciFunctions._;
import scala.collection.mutable.HashMap;

class MinitaurEval(val p:Bullet, val urdfRootPath:String = "") {

    var minitaur:Minitaur = null;
    
    val evaluate_func_map = new HashMap[String,(Int,DMat)=>DMat]();

    def current_position() = {
	minitaur.getBasePosition();
    };

    def is_fallen() = {
	val orientation = minitaur.getBaseOrientation();
	val rotMat = p.getMatrixFromQuaternion(orientation);
	val localUp = rotMat(2,?);
	val test = ((row(0,0,1) dotr localUp).v < 0);
	test;
    };

    def evaluate_desired_motorAngle_8Amplitude8Phase(i:Int, params:DMat) = {
	val nMotors = 8;
	val speed = 0.35;
	val joint_values = dzeros(1, nMotors);
	for (jthMotor <- 0 until nMotors) {
	    joint_values(jthMotor) = math.sin(i*speed + params(nMotors + jthMotor))*params(jthMotor)*1.57;
	}
	joint_values;
    };

    def evaluate_desired_motorAngle_2Amplitude4Phase(i:Int, params:DMat) = {
	val speed = 0.35;
	val phaseDiff = params(2);
	val a0 = math.sin(i * speed) * params(0) + 1.57;
	val a1 = math.sin(i * speed + phaseDiff) * params(1) + 1.57;
	val a2 = math.sin(i * speed + params(3)) * params(0) + 1.57;
	val a3 = math.sin(i * speed + params(3) + phaseDiff) * params(1) + 1.57;
	val a4 = math.sin(i * speed + params(4) + phaseDiff) * params(1) + 1.57;
	val a5 = math.sin(i * speed + params(4)) * params(0) + 1.57;
	val a6 = math.sin(i * speed + params(5) + phaseDiff) * params(1) + 1.57;
	val a7 = math.sin(i * speed + params(5)) * params(0) + 1.57;
	val joint_values = drow(a0, a1, a2, a3, a4, a5, a6, a7);
	joint_values;
    };

    def evaluate_desired_motorAngle_hop(i:Int, params:DMat) = {
	val amplitude = params(0);
	val speed = params(1);
	val a1 = math.sin(i*speed)*amplitude+1.57;
	val a2 = math.sin(i*speed+3.14)*amplitude+1.57;
	val joint_values = drow(a1, 1.57, a2, 1.57, 1.57, a1, 1.57, a2);
	joint_values;
    };


    evaluate_func_map("evaluate_desired_motorAngle_8Amplitude8Phase") = evaluate_desired_motorAngle_8Amplitude8Phase;
    evaluate_func_map("evaluate_desired_motorAngle_2Amplitude4Phase") = evaluate_desired_motorAngle_2Amplitude4Phase;
    evaluate_func_map("evaluate_desired_motorAngle_hop") = evaluate_desired_motorAngle_hop;



    def evaluate_params(evaluateFunc:String, params:DMat, objectiveParams:DMat, timeStep:Double=0.01, maxNumSteps:Int=10000, sleepTime:Double=0) = {
	print("start evaluation");
	val beforeTime = System.currentTimeMillis()/1000.0;
	p.resetSimulation();

	p.setTimeStep(timeStep);
	p.loadURDF("%s/plane.urdf" format urdfRootPath);
	p.setGravity(0,0,-10);

	minitaur = new Minitaur(p, urdfRootPath);
	val start_position = current_position();
	var last_position:FMat = null // for tracing line
	var total_energy = 0.0;

	var done = false;
	var i = 0;
	while (i < maxNumSteps && !done) {
	    val torques = minitaur.getMotorTorques();
	    val velocities = minitaur.getMotorVelocities();
	    total_energy += (abs(torques) dotr abs(velocities)).v * timeStep;

	    val joint_values = evaluate_func_map(evaluateFunc)(i, params);
	    minitaur.applyAction(joint_values);
	    p.stepSimulation();
	    if (is_fallen()) {
		done = true;
	    }
	    if (i % 100 == 0) {
		print(".");
		System.out.flush();
	    }
	    Thread.sleep((sleepTime*1000).toLong);
	    i += 1;
	}

	println(" "); 

	val alpha = objectiveParams(0);
	val final_distance = norm(start_position - current_position());
	val finalReturn = final_distance - alpha * total_energy;
	val elapsedTime = System.currentTimeMillis()/1000.0 - beforeTime;
	println ("trial for ", params, " final_distance", final_distance, "total_energy", total_energy, "finalReturn", finalReturn, "elapsed_time", elapsedTime);
	finalReturn
    }
}

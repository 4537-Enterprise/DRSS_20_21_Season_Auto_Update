package org.firstinspires.ftc.teamcode.Competition_Code.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Competition_Code.InitHardware;
import org.firstinspires.ftc.teamcode.Competition_Code.TeleOp.Scorpion;
import org.firstinspires.ftc.teamcode.Competition_Code.encoders;

import java.util.List;

@Autonomous(name = "ScorpionAuto", group = "Auto")
public class ScorpionAuto extends LinearOpMode{

	private InitHardware robot = new InitHardware();  	//Load hardware from hardware map

	private int newLeftTarget;
	private int newRightTarget;
	private int newCenterTarget;

	private String ringCount = null;

	private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
	private static final String LABEL_FIRST_ELEMENT = "Quad";
	private static final String LABEL_SECOND_ELEMENT = "Single";

	/*
	 * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
	 * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
	 * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
	 * web site at https://developer.vuforia.com/license-manager.
	 *
	 * Vuforia license keys are always 380 characters long, and look as if they contain mostly
	 * random data. As an example, here is a example of a fragment of a valid key:
	 *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
	 * Once you've obtained a license key, copy the string from the Vuforia web site
	 * and paste it in to your code on the next line, between the double quotes.
	 */
	public static final String VUFORIA_KEY = "AUz/xEr/////AAABmZjEf3Mc1kGYkOsXVF3u1Gl8gO6qZozGZxty6mcO/xE35elxxgMBwh4/zzwC9Dh4EPKvDexbQAVpjQJzz+Cx+PMYbKiPfvJNsyHDoJkWCPC1skmjKJq/4ctLkD1zGtWPhVUsdGK9ib6ze346j5nHgoFwzoi4SAITZUfQZEj2ccyiWs3zvY2DzbL/QgXrk391epqrpmB6y96vnvCsTUYA6i1y8pg7TZmjUBNWC/3PMr0EHBAFzu+cgtMWVD2sjR9XYcyh9eCRKFNq1aZwikL2P2F4Px5eyujkCVBsnQ0N+dNBo/UCREIF2az5iJY/x+qnrr8aZ2Rj1Gri12gHuKLT7BWS73HKsC9XVURurHz9RmJs";
	/**
	 * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
	 * localization engine.
	 */
	private VuforiaLocalizer vuforia;

	/**
	 * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
	 * Detection engine.
	 */
	private TFObjectDetector tfod;

	private int step = 1;

	@Override
	public void runOpMode() throws InterruptedException{

		robot.init(hardwareMap); //Initialize hardware
		telemetry.addData("Drive Train", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.update();

		robot.angleAdjustLeft.setPosition(robot.anglePositionLeft); 	//Start servos at lowest point
		robot.angleAdjustRight.setPosition(robot.anglePositionRight); //Start servos at lowest point
		telemetry.addData("Drive Train", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Angle Adjust", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Tensor Flow Object Detection", "Initializing");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.update();

		// The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
		// first.
		initVuforia();
		initTfod();

		/**
		 * Activate TensorFlow Object Detection before we wait for the start command.
		 * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
		 **/
		if (tfod != null) {
			tfod.activate();

			// The TensorFlow software will scale the input images from the camera to a lower resolution.
			// This can result in lower detection accuracy at longer distances (> 55cm or 22").
			// If your target is at distance greater than 50 cm (20") you can adjust the magnification value
			// to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
			// should be set to the value of the images used to create the TensorFlow Object Detection model
			// (typically 16/9).
			tfod.setZoom(1.5, 16.0/9.0);
		}

		telemetry.addData("Drive Train", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Angle Adjust", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Tensor Flow Object Detection", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.update();

		robot.flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.boreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		robot.launch.setVelocity(0);				//Start up launcher
		robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		robot.gripper.setPosition(0);

		telemetry.addData("Drive Train", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Angle Adjust", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Tensor Flow Object Detection", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Launcher", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Status", "Ready");                 // Adds telemetry to the screen to show that the robot is ready
		telemetry.addData("Press Play to Start", "Autonomous");    // Adds telemetry to the screen to tell the drivers that the code is ready to start
		telemetry.update();

		waitForStart();

		if (step == 1) {
			drive(.5,1,1); // Move forward 1 inch
			sleep(100);
			turn(.25,4,1); // Turn right 5 degrees
			step++;
		}

		if (step == 2) {
			autoAim(107,3); // Shoot three rings
			turn(.25,-3,-1);
			step++;
		}

		if (step == 3) {
			// getUpdatedRecognitions() will return null if no new information is available since
			// the last time that call was made.
			List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
			if (updatedRecognitions != null) {
				if (updatedRecognitions.size() == 0) {
					ringCount = null;
					step++;
				}
				// step through the list of recognitions and display boundary info.
				int i = 0;
				for (Recognition recognition : updatedRecognitions) {
					ringCount = recognition.getLabel();
					step++;
				}
			}
		}

		if (step == 4 && ringCount == "Quad") {
			quad();
		}

		if (step == 4 && ringCount == "Single") {
			single();
		}

		if (step == 4 && ringCount == null) {
			none();
		}
	}

	/**
	 * Alternate Ring Position Voids
	 */
	private void quad() throws InterruptedException{
		if (step == 4) {
			drive(.5,22,1);
			step++;
		}

		if (step == 5) {
			robot.intakeMotor.setPower(-1);

			robot.anglePositionLeft = .26;
			robot.anglePositionRight = .74;
			robot.angleAdjustLeft.setPosition(robot.anglePositionLeft);  	//Set Servo Position
			robot.angleAdjustRight.setPosition(robot.anglePositionRight);   //Set Servo Position

			drive(.25,15,1);
			drive(.15,5,1);
			sleep(2250);

			step++;
		}

		if (step == 6) {
			turn(1,4,1);

			robot.zeroLauncherAngle();
			autoAim(78,2);

			step++;
		}

		if (step == 7) {
			turn(1,-4,-1);
			drive(.25,20,1);

			step++;
		}

		if (step == 8) {
			drive(1,30,1);
			turn(1,-30,-1);

			step++;
		}

		if (step == 9) {
			armDown();
			robot.gripper.setPosition(1);
			sleep(100);

			step++;
		}

		if (step == 10) {
			drive(1,-25,-1);
			armUp();

			step++;
		}

	}

	private void single() {
		if (step == 4) {
			drive(.5,22,1);
			step++;
		}

		if (step == 5) {
			robot.intakeMotor.setPower(-1);

			robot.anglePositionLeft = .26;
			robot.anglePositionRight = .74;
			robot.angleAdjustLeft.setPosition(robot.anglePositionLeft);  	//Set Servo Position
			robot.angleAdjustRight.setPosition(robot.anglePositionRight);   //Set Servo Position

			drive(.25,15,1);

			step++;
		}

		if (step == 6) {
			turn(.5,.5,1);
			drive(.5,30,1);

			step++;
		}

		if (step == 7) {
			armDown();
			robot.gripper.setPosition(1);
			sleep(100);

			step++;
		}

		if (step == 8) {
			drive(1,-2,-1);
			turn(.5,-180,-1);
			sleep(500);

			step++;
		}

		if (step == 9) {
			drive(.5,32,1);
			drive(.15,4,1);

			robot.intakeMotor.setPower(0);
			robot.gripper.setPosition(0);
			sleep(250);

			step++;
		}

		if (step == 10) {
			armUp();

			step++;
		}

		if (step == 11) {
			turn(.5,185,1);
			drive(.5,45,1);

			step++;
		}

		if (step == 12) {
			armDown();
			robot.gripper.setPosition(1);

			drive(1,-4,-1);

			step++;
		}
	}

	private void none() {
		if (step == 4) {
			drive(.5,40,1);
			turn(.5,-45,-1);
			drive(.5,12,1);

			step++;
		}

		if (step == 5) {
			armDown();

			robot.gripper.setPosition(1);
			sleep(250);

			step++;
		}

		if (step == 6) {
			drive(.5, -2,-1);
			turn(.5,-145,-1);

			step++;
		}

		if (step == 7) {
			drive(.5,16,1);
			drive(.15,2,1);

			robot.gripper.setPosition(0);
			sleep(250);

			step++;
		}

		if (step == 8) {
			armUp();
			sleep(500);

			turn(.5,145,1);
			drive(.5,18,1);

			step++;
		}

		if (step == 9) {
			armDown();

			strafe(1,12,1);
			drive(1,8,1);

			step++;
		}
	}

	/**
	 * Voids for moving, turning, and strafing using odometers.
	 */

	public void drive (double speed, double Inches, int directionModifier) {

		// Ensure that the opmode is still active
		if (opModeIsActive()){

			// Math to calculate each target position for the motors
			newLeftTarget = robot.leftEncoder.getCurrentPosition() + (int) ((Inches) * robot.COUNTS_PER_INCH);
			newRightTarget = robot.rightEncoder.getCurrentPosition() + (int) ((Inches) * robot.COUNTS_PER_INCH);

			// reset the timeout time and start motion.
			robot.motorFrontLeft.setPower(speed*directionModifier);
			robot.motorFrontRight.setPower(speed*directionModifier);
			robot.motorBackLeft.setPower(speed*directionModifier);
			robot.motorBackRight.setPower(speed*directionModifier);

			// keep looping while we are still active, and there is time left, and both motors are running.
			// Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
			// its target position, the motion will stop.  This is "safer" in the event that the robot will
			// always end the motion as soon as possible.
			// However, if you require that BOTH motors have finished their moves before the robot continues
			// onto the next step, use (isBusy() || isBusy()) in the loop test.
			while (opModeIsActive() && (leftIsBusy(directionModifier) && rightIsBusy(directionModifier))){

				// Display it for the driver.
				telemetry.addData("Motor Paths", "Running at %7d : %7d", //Tells us where we are
						robot.leftEncoder.getCurrentPosition(), //Left Position
						robot.rightEncoder.getCurrentPosition()); //Right Position
				telemetry.update();
			}

			// Stop all motion;
			robot.stopMotors();
		}

	}

	public void turn (double speed, double Angle, int directionModifier) {

		double c = 40.84; //Circumference of arc created by robot wheels (Radius is 9.605")
		double ANGLE_RATIO = Angle / 360; //Ratio of angle relative to entire circle
		double CIRCUMFERENCE_OF_ANGLE = c * ANGLE_RATIO; //Circumference of Angle
		int COUNTS_PER_DISTANCE = (int) ((CIRCUMFERENCE_OF_ANGLE * robot.COUNTS_PER_INCH));

		// Ensure that the opmode is still active
		if (opModeIsActive()){

			// Math to calculate each target position for the motors
			//newLeftTarget = robot.leftEncoder.getCurrentPosition() + (int) (Angle * robot.COUNTS_PER_DEGREE);
			//newRightTarget = robot.rightEncoder.getCurrentPosition() - (int) (Angle * robot.COUNTS_PER_DEGREE);
			newLeftTarget = robot.leftEncoder.getCurrentPosition() + COUNTS_PER_DISTANCE;
			newRightTarget = robot.rightEncoder.getCurrentPosition() - COUNTS_PER_DISTANCE;

			// reset the timeout time and start motion.
			robot.motorFrontLeft.setPower(speed*directionModifier);
			robot.motorFrontRight.setPower(-speed*directionModifier);
			robot.motorBackLeft.setPower(speed*directionModifier);
			robot.motorBackRight.setPower(-speed*directionModifier);

			// keep looping while we are still active, and there is time left, and both motors are running.
			// Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
			// its target position, the motion will stop.  This is "safer" in the event that the robot will
			// always end the motion as soon as possible.
			// However, if you require that BOTH motors have finished their moves before the robot continues
			// onto the next step, use (isBusy() || isBusy()) in the loop test.
			while (opModeIsActive() && (leftIsBusy(directionModifier) && rightIsBusy(-directionModifier))){

				// Display it for the driver.
				telemetry.addData("Motor Paths", "Running at %7d : %7d", //Tells us where we are
						robot.leftEncoder.getCurrentPosition(), //Left Position
						robot.rightEncoder.getCurrentPosition()); //Right Position
				telemetry.update();
			}

			// Stop all motion;
			robot.stopMotors();
		}

	}

	public void strafe (double speed, double Inches, int directionModifier) {

		// Ensure that the opmode is still active
		if (opModeIsActive()){

			// Math to calculate each target position for the motors
			newCenterTarget = robot.centerEncoder.getCurrentPosition() + (int) (Inches * robot.COUNTS_PER_INCH);

			// reset the timeout time and start motion.
			robot.motorFrontLeft.setPower(speed*directionModifier);
			robot.motorFrontRight.setPower(-speed*directionModifier);
			robot.motorBackLeft.setPower(-speed*directionModifier);
			robot.motorBackRight.setPower(speed*directionModifier);

			// keep looping while we are still active, and there is time left, and both motors are running.
			// Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
			// its target position, the motion will stop.  This is "safer" in the event that the robot will
			// always end the motion as soon as possible.
			// However, if you require that BOTH motors have finished their moves before the robot continues
			// onto the next step, use (isBusy() || isBusy()) in the loop test.
			while (opModeIsActive() && (centerIsBusy(directionModifier))){

				// Display it for the driver.
				telemetry.addData("Motor Paths", "Running at %7d", //Tells us where we are
						robot.centerEncoder.getCurrentPosition()); //Left Position
				telemetry.update();
			}

			// Stop all motion;
			robot.stopMotors();
		}

	}

	/**
	 * Voids for checking if the odometers are busy or not
	 */

	private boolean leftIsBusy(int direction) {

		if (direction == 1) { 																		// If we are moving forward
			if ((robot.leftEncoder.getCurrentPosition()) >= newLeftTarget) { 		// Check if we have passed our target
				return false; 																		// Send stop command
			}
			else { 																					// If we haven't passed our target
				return true; 																		// Send continue command
			}
		}

		if (direction == -1) { // If we are moving backwards
			if ((robot.leftEncoder.getCurrentPosition()) <= newLeftTarget) { 		// Check if we have passed our target
				return false; 																		// Send stop command
			}
			else { 																					// If we haven't passed our target
				return true; 																		// send continue command
			}
		}
		return false; 																				// If no direction was specified, tell the code to stop
	}

	private boolean rightIsBusy(int direction) {

		if (direction == 1) { 																		// If we are moving forward
			if ((robot.rightEncoder.getCurrentPosition()) >= newRightTarget) { 	// Check if we have passed our target
				return false; 																		// Send stop command
			}
			else { 																					// If we haven't passed our target
				return true; 																		// Send continue command
			}
		}

		if (direction == -1) { 																		// If we are moving backwards
			if ((robot.rightEncoder.getCurrentPosition()) <= newRightTarget) { 	// Check if we have passed our target
				return false; 																		// Send stop command
			}
			else { 																					// If we haven't passed our target
				return true; 																		// send continue command
			}
		}
		return false; 																				// If no direction was specified, tell the code to stop
	}

	private boolean centerIsBusy(int direction) {

		if (direction == 1) { 																		// If we are moving forward
			if ((robot.centerEncoder.getCurrentPosition()) >= newCenterTarget) { // Check if we have passed our target
				return false; 																		// Send stop command
			}
			else { 																					// If we haven't passed our target
				return true; 																		// Send continue command
			}
		}

		if (direction == -1) { 																		// If we are moving backwards
			if ((robot.centerEncoder.getCurrentPosition()) <= newCenterTarget) { // Check if we have passed our target
				return false; 																		// Send stop command
			}
			else { 																					// If we haven't passed our target
				return true; 																		// send continue command
			}
		}
		return false; 																				// If no direction was specified, tell the code to stop
	}

	/**
	 * Voids for aiming and launching rings
	 */

	public void autoAim(float distance, int rings) throws InterruptedException{
		if (distance == 0) {
			return;
		}
		robot.stopMotors();
		float angle = (float) ((0.001*Math.pow((distance-60),2))+15.25);
		robot.expectedAngle = angle;
		robot.launch.setVelocity(2800);
		sleep(750);
		robot.setLauncherAngle(angle);
		robot.actualAngle = (robot.boreEncoder.getCurrentPosition()/robot.countsPerDegree);
		robot.launch(rings);
		robot.launch.setVelocity(0);
		robot.zeroLauncherAngle();
	}

	public void armDown() {
		robot.stopMotors();
		robot.armDown = true;
		robot.arm.setTargetPosition(115);
		robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.arm.setPower(1);
		while (robot.arm.isBusy()) {
			if (gamepad2.back) {
				robot.gripper.setPosition(.65);
				sleep(250);
				robot.gripper.setPosition(1);
				robot.gripperOpen = true;
				return;
			}
		}
		robot.arm.setPower(0);
		robot.gripper.setPosition(1);
		robot.gripperOpen = true;
	}

	public void armUp() {
		robot.stopMotors();
		robot.arm.setTargetPosition(15);
		robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.arm.setPower(1);
		while (robot.arm.isBusy()) {
			if (gamepad2.back) {
				return;
			}
		}
		robot.arm.setPower(0);
		robot.armDown = false;
	}

	/**
	 * Initialize the Vuforia localization engine.
	 */
	private void initVuforia() {
		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 */
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

		//  Instantiate the Vuforia engine
		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		// Loading trackables is not necessary for the TensorFlow Object Detection engine.
	}

	/**
	 * Initialize the TensorFlow Object Detection engine.
	 */
	private void initTfod() {
		int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
				"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		tfodParameters.minResultConfidence = 0.8f;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
	}
}

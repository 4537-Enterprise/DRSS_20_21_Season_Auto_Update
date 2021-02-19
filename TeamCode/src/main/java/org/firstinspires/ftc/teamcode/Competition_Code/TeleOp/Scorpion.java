package org.firstinspires.ftc.teamcode.Competition_Code.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Competition_Code.InitHardware;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name = "Scorpion", group = "TeleOp")
public class Scorpion extends LinearOpMode{

	private InitHardware robot = new InitHardware();  //Load hardware from hardware map
	FtcDashboard dashboard = FtcDashboard.getInstance();
	TelemetryPacket packet = new TelemetryPacket();

	WebcamName webcamName = null;

	@Override
	public void runOpMode() throws InterruptedException{

		robot.init(hardwareMap); //Initialize hardware
		telemetry.addData("Drive Train", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.update();

		robot.angleAdjustLeft.setPosition(robot.anglePositionLeft); 	//Start servos at lowest point
		robot.angleAdjustRight.setPosition(robot.anglePositionRight); //Start servos at lowest point
		telemetry.addData("Drive Train", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Angle Adjust", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Vuforia", "Initializing");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.update();

		/* Vuforia Initializations*/
			webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

			int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
			VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

			parameters.vuforiaLicenseKey = robot.VUFORIA_KEY;

			parameters.cameraName = webcamName;

			// Make sure extended tracking is disabled for this example.
			parameters.useExtendedTracking = false;

			//  Instantiate the Vuforia engine
			robot.vuforia = ClassFactory.getInstance().createVuforia(parameters);

			// Load the data sets for the trackable objects. These particular data
			// sets are stored in the 'assets' part of our application.
			VuforiaTrackables targetsUltimateGoal = this.robot.vuforia.loadTrackablesFromAsset("UltimateGoal");
			VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
			blueTowerGoalTarget.setName("Blue Tower Goal Target");
			VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
			redTowerGoalTarget.setName("Red Tower Goal Target");
			VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
			redAllianceTarget.setName("Red Alliance Target");
			VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
			blueAllianceTarget.setName("Blue Alliance Target");
			VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
			frontWallTarget.setName("Front Wall Target");

			// For convenience, gather together all the trackable objects in one easily-iterable collection */
			List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
			allTrackables.addAll(targetsUltimateGoal);

		redAllianceTarget.setLocation(OpenGLMatrix
				.translation(0, -robot.halfField, robot.mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

		blueAllianceTarget.setLocation(OpenGLMatrix
				.translation(0, robot.halfField, robot.mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
		frontWallTarget.setLocation(OpenGLMatrix
				.translation(-robot.halfField, 0, robot.mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

		// The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
		blueTowerGoalTarget.setLocation(OpenGLMatrix
				.translation(robot.halfField, robot.quadField, robot.mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
		redTowerGoalTarget.setLocation(OpenGLMatrix
				.translation(robot.halfField, -robot.quadField, robot.mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

			// We need to rotate the camera around it's long axis to bring the correct camera forward.
			if (robot.CAMERA_CHOICE == BACK) {
				robot.phoneYRotate = -90;
			} else {
				robot.phoneYRotate = 90;
			}

			// Rotate the phone vertical about the X axis if it's in portrait mode
			if (robot.PHONE_IS_PORTRAIT) {
				robot.phoneXRotate = 90 ;
			}

			// Next, translate the camera lens to where it is on the robot.
			// In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
			final float CAMERA_FORWARD_DISPLACEMENT  = 8.0f * robot.mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
			final float CAMERA_VERTICAL_DISPLACEMENT = 4.5f * robot.mmPerInch;   // eg: Camera is 8 Inches above ground
			final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

			OpenGLMatrix robotFromCamera = OpenGLMatrix
					.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
					.multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, robot.phoneYRotate, robot.phoneZRotate, robot.phoneXRotate));

			/**  Let all the trackable listeners know where the phone is.  */
			for (VuforiaTrackable trackable : allTrackables) {
				((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
			}
		/* End of Vuforia Initializations */
		telemetry.addData("Drive Train", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Angle Adjust", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Vuforia", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.update();

		/* Start of Tensor Flow Initializations*/
			int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
					"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
			TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
			tfodParameters.minResultConfidence = 0.8f;
			robot.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, robot.vuforia);
			robot.tfod.loadModelFromAsset(robot.TFOD_MODEL_ASSET, robot.LABEL_FIRST_ELEMENT, robot.LABEL_SECOND_ELEMENT);

			//robot.activateTFOD();

		robot.flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.boreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		robot.launch.setVelocity(0);				//Start up launcher
		robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		packet.put("Program is", "Ready");
		dashboard.sendTelemetryPacket(packet);

		telemetry.addData("Drive Train", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Angle Adjust", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Vuforia", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Launcher", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Status", "Ready");                 // Adds telemetry to the screen to show that the robot is ready
		telemetry.addData("Press Play to Start", "TeleOp");    // Adds telemetry to the screen to tell the drivers that the code is ready to start
		telemetry.update();

		waitForStart();

		targetsUltimateGoal.activate();

		while (opModeIsActive()){

			/**Mechanum drive controls**/
				// left stick controls direction
				// right stick X controls rotation
				float gamepad1LeftY = -gamepad1.left_stick_y;        // Sets the gamepads left sticks y position to a float so that we can easily track the stick
				float gamepad1LeftX = gamepad1.left_stick_x;       // Sets the gamepads left sticks x position to a float so that we can easily track the stick
				float gamepad1RightX = -gamepad1.right_stick_x;     // Sets the gamepads right sticks x position to a float so that we can easily track the stick

				// Mechanum formulas
				/*double FrontRight = gamepad1LeftX - gamepad1LeftY + gamepad1RightX;     // Combines the inputs of the sticks to clip their output to a value between 1 and -1
				double FrontLeft = -gamepad1LeftX - gamepad1LeftY - gamepad1RightX;     // Combines the inputs of the sticks to clip their output to a value between 1 and -1
				double BackRight = gamepad1LeftX + gamepad1LeftY + gamepad1RightX;      // Combines the inputs of the sticks to clip their output to a value between 1 and -1
				double BackLeft = -gamepad1LeftX + gamepad1LeftY - gamepad1RightX;      // Combines the inputs of the sticks to clip their output to a value between 1 and -1
				*/
				double FrontRight = gamepad1LeftX + gamepad1LeftY - gamepad1RightX;     // Combines the inputs of the sticks to clip their output to a value between 1 and -1
				double FrontLeft = -gamepad1LeftX + gamepad1LeftY + gamepad1RightX;     // Combines the inputs of the sticks to clip their output to a value between 1 and -1
				double BackRight = -gamepad1LeftX + gamepad1LeftY - gamepad1RightX;      // Combines the inputs of the sticks to clip their output to a value between 1 and -1
				double BackLeft = gamepad1LeftX + gamepad1LeftY + gamepad1RightX;      // Combines the inputs of the sticks to clip their output to a value between 1 and -1

				// sets speed
				robot.frontLeft = Range.clip(Math.pow(FrontRight, 3), -robot.speed, robot.speed);    // Slows down the motor and sets its max/min speed to the double "speed"
				robot.frontRight = Range.clip(Math.pow(FrontLeft, 3), -robot.speed, robot.speed);      // Slows down the motor and sets its max/min speed to the double "speed"
				robot.backLeft = Range.clip(Math.pow(BackRight, 3), -robot.speed, robot.speed);      // Slows down the motor and sets its max/min speed to the double "speed"
				robot.backRight = Range.clip(Math.pow(BackLeft, 3), -robot.speed, robot.speed);        // Slows down the motor and sets its max/min speed to the double "speed"

				//speed Controls
				if (gamepad1.left_trigger > .3){   // Do the following while the left trigger is being held down
					robot.speed = .25;                    // Sets the speed to quarter speed
				}

				if (gamepad1.right_trigger < .3 && gamepad1.left_trigger < .3){   // Do the following  while the right trigger is held down and the left trigger is not
					robot.speed = 1;                     // Sets the speed to half speed
				}

				if (gamepad1.right_trigger > .3){    // Do the following while the left trigger is not being held down
					robot.speed = .5;
				}

				robot.motorFrontRight.setPower(robot.frontRight);   // Sets the front right motors speed to the previous double
				robot.motorFrontLeft.setPower(robot.frontLeft);     // Sets the front left motors speed to the previous double
				robot.motorBackRight.setPower(robot.backRight);     // Sets the back right motors speed to the previous double
				robot.motorBackLeft.setPower(robot.backLeft);       // Sets the back left motors speed to the previous double
				//End of speed Controls
			/**End of Mechanum Controls**/

			/**Intake Controls**/
				if (gamepad2.left_trigger > .25) {
					robot.intakeMotor.setPower(-1); //Start running intake motor
					robot.anglePositionLeft = .26;
					robot.anglePositionRight = .74;
				}
				else {
					robot.intakeMotor.setPower(0); //Stop running intake motor
				}
			/**End of intake controls**/

			/**Launcher Controls**/
				if (gamepad2.dpad_up && robot.anglePositionLeft >= 0 && robot.anglePositionRight <= 1) {	//Move the launcher up
					robot.anglePositionLeft = robot.anglePositionLeft - 0.005;  								//Subtract from current angle on servo
					robot.anglePositionRight = robot.anglePositionRight + 0.005;								//Add from current angle on servo
					//robot.setLauncherAngle(25);
				}

				if (gamepad2.dpad_down  && robot.anglePositionRight >= .34 && robot.anglePositionLeft <= .66) {
					//Move the launcher down
					//robot.anglePositionLeft = robot.anglePositionLeft + 0.005;  	//Add from current angle on servo
					//robot.anglePositionRight = robot.anglePositionRight - 0.005;	//Subtract from current angle on servo
					//robot.zeroLauncherAngle();
					robot.anglePositionLeft = .66;
					robot.anglePositionRight = .34;
				}

				robot.angleAdjustLeft.setPosition(robot.anglePositionLeft);  	//Set Servo Position
				robot.angleAdjustRight.setPosition(robot.anglePositionRight);   //Set Servo Position
				robot.launcherAngle = (int) (robot.boreEncoder.getCurrentPosition()/robot.countsPerDegree);

				if (gamepad2.x) {				//Ramp up launcher
					 robot.launch.setVelocity(2800);  //Set launcher motor to full speed
				}
				else {							//Slow down launcher when not shooting
					robot.launch.setVelocity(0);	//Set launcher motor to half speed
				}

				if (gamepad2.right_trigger > .25) { //Activate Pusher
					autoAim(robot.distanceFromTarget);
					//launch(3);
				}
				if (gamepad2.right_bumper) { //Activate Push+er
					powerShot(74);
				}
			/**End of launcher controls**/

			/**Wobble Goal Arm Controls**/
				if (gamepad2.a && robot.armDown) {
					armUp();
				}
				else if (gamepad2.a) {
					armDown();
				}
				else {
					robot.arm.setPower(0);
				}

				if (gamepad2.b && robot.gripperOpen) {
					robot.gripper.setPosition(0);
					robot.gripperOpen = false;
				}
				else if (gamepad2.y) {
					robot.gripper.setPosition(1);
					robot.gripperOpen = true;
				}

			/**Vuforia**/
				// check all the trackable targets to see which one (if any) is visible.
				robot.targetVisible = false;
				for (VuforiaTrackable trackable : allTrackables) {
					if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
						robot.targetVisible = true;
						robot.targetName = trackable.getName();

						// getUpdatedRobotLocation() will return null if no new information is available since
						// the last time that call was made, or if the trackable is not currently visible.
						OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
						if (robotLocationTransform != null) {
							robot.lastLocation = robotLocationTransform;
						}
						break;
					}
				}

			/**Object detection testing code**/
				if (robot.targetVisible && (robot.targetName == "Blue Tower Goal Target" || robot.targetName == "Red Tower Goal Target")) {
					// express position (translation) of robot in inches.
					VectorF translation = robot.lastLocation.getTranslation();
					robot.distanceFromTarget = -((translation.get(0) / robot.mmPerInch) - 72);
					robot.x = translation.get(0) / robot.mmPerInch;
					robot.y = translation.get(1) / robot.mmPerInch;

					// express the rotation of the robot in degrees.
					Orientation rotation = Orientation.getOrientation(robot.lastLocation, EXTRINSIC, XYZ, DEGREES);
					robot.heading = rotation.thirdAngle;

					telemetry.addData("Visible Target", robot.targetName);
					telemetry.addData("Pos (inch)", "Distance From Target = %.0f", robot.distanceFromTarget);
					telemetry.addData("Pos (inch)", "X, Y = %.0f, %.0f", robot.x, robot.y);
					telemetry.addData("Rot (deg)", "Heading = %.0f", robot.heading);

					packet.put("Visible Target", robot.targetName);
					packet.put("Distance from Target", robot.distanceFromTarget);
					packet.put("Pos (inch) X", robot.x);
					packet.put("pos (inch) Y", robot.y);
					packet.put("Rot (deg)", robot.heading);
				}
				else if (robot.targetVisible) {
					// express position (translation) of robot in inches.
					VectorF translation = robot.lastLocation.getTranslation();
					robot.x = translation.get(0) / robot.mmPerInch;
					robot.y = translation.get(1) / robot.mmPerInch;
					robot.distanceFromTarget = 0;

					// express the rotation of the robot in degrees.
					Orientation rotation = Orientation.getOrientation(robot.lastLocation, EXTRINSIC, XYZ, DEGREES);
					robot.heading = rotation.thirdAngle;

					telemetry.addData("Visible Target", robot.targetName);
					telemetry.addData("Pos (inch)", "X, Y = %.0f, %.0f", robot.x, robot.y);
					telemetry.addData("Rot (deg)", "Heading = %.0f", robot.heading);

					packet.put("Visible Target", robot.targetName);
					packet.put("Pos (inch) X", robot.x);
					packet.put("pos (inch) Y", robot.y);
					packet.put("Rot (deg)", robot.heading);
				}
				else {
					robot.targetName = null;
					robot.distanceFromTarget = 0;
					telemetry.addData("Visible Target", "None");
					packet.put("Visible Target", "None");
				}

			/**TFOD Code**/

			/*if (robot.tfod != null) {
				// getUpdatedRecognitions() will return null if no new information is available since
				// the last time that call was made.
				List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
				if (updatedRecognitions != null) {
					telemetry.addData("# Object Detected", updatedRecognitions.size());
					// step through the list of recognitions and display boundary info.
					int i = 0;
					for (Recognition recognition : updatedRecognitions) {
						telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
						telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
								recognition.getLeft(), recognition.getTop());
						telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
								recognition.getRight(), recognition.getBottom());
					}
				}
			}*/

			/**Telemetry**/
				telemetry.addData("","");
				telemetry.addData("Launcher Angle", robot.launcherAngle);
				telemetry.addData("Speed", robot.speed);
				telemetry.update();

				packet.put("", "");
				packet.put("Launcher Angle", robot.launcherAngle);
				packet.put("Speed", robot.speed);
				dashboard.sendTelemetryPacket(packet);
			/**End of telemetry**/
		}

		targetsUltimateGoal.deactivate();
		//robot.deactivateTFOD();
	}

	public void autoAim(float distance) throws InterruptedException{
		if (distance == 0) {
			return;
		}
		robot.stopMotors();
		float angle = (float) ((0.001*Math.pow((distance-60),2))+17.5);
		robot.launch.setVelocity(2800);
		sleep(700);
		robot.setLauncherAngle(angle);
		launch(3);
		robot.launch.setVelocity(0);
		robot.zeroLauncherAngle();
	}

	public void powerShot(float distance) throws InterruptedException{
		robot.stopMotors();
		float angle = (float) ((0.001*Math.pow((distance-60),2))+17.5);
		robot.launch.setVelocity(2800);
		sleep(700);
		robot.setLauncherAngle(angle);
		launch(1);
		robot.launch.setVelocity(0);
		robot.zeroLauncherAngle();
	}

	/** Auto Aim Voids so I can kill the program **/
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

	/** Auto Aim Voids so I can kill the program **/
	public void launch(int rings) throws InterruptedException{
		for (int i = 1; i <= rings; i++) {
			robot.flipper.setTargetPosition(65);
			robot.flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			robot.flipper.setPower(1);
			while (robot.flipper.isBusy()) {
				if (gamepad2.back) {
					robot.flipper.setTargetPosition(-2);
					while (robot.flipper.isBusy()) {
					}
					robot.flipper.setPower(0);
					break;
				}
			}
			robot.flipper.setTargetPosition(-2);
			while (robot.flipper.isBusy()) {
				if (gamepad2.back) {
					robot.flipper.setTargetPosition(-2);
					while (robot.flipper.isBusy()) {
					}
					robot.flipper.setPower(0);
					break;
				}
			}
			robot.flipper.setPower(0);
			//sleep(400);
		}
	}

}

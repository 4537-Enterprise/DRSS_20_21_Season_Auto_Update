package org.firstinspires.ftc.teamcode.Competition_Code.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Competition_Code.InitHardware;
import org.firstinspires.ftc.teamcode.Competition_Code.InitHardwareRoadRunner;
import org.firstinspires.ftc.teamcode.Competition_Code.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Competition_Code.RoadRunner.drive.SampleMecanumDrive;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "ScorpionAutoRoadRunner", group = "Auto")
public class ScorpionAutoRoadRunner extends LinearOpMode{

	private InitHardwareRoadRunner robot = new InitHardwareRoadRunner();  	//Load hardware from hardware map

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

	int step = 0;
	String ringCount = "";

	@Override
	public void runOpMode() throws InterruptedException{

		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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
		telemetry.addData("Trajectories", "Initializing");
		telemetry.update();

		// We want to start the bot at x: -63, y: 18, heading: 0 degrees
		Pose2d startPose = new Pose2d(-63, 32, Math.toRadians(0.0));
		drive.setPoseEstimate(startPose);

		/**Quad Ring Trajectory Builders**/

			Trajectory quadTraj1 = drive.trajectoryBuilder(startPose, false)
					.splineTo(new Vector2d(-6, 10), Math.toRadians(0.0))
					.splineTo(new Vector2d(50,40), Math.toRadians(80.0))
					.build();

			Trajectory quadTraj2 = drive.trajectoryBuilder(quadTraj1.end(), true)
					.splineTo(new Vector2d(-2, 32), Math.toRadians(180.0))
					.addDisplacementMarker(7, () -> {
						robot.launch.setVelocity(3000);
					})
					.build();

			Trajectory quadTraj3 = drive.trajectoryBuilder(quadTraj2.end(), false)
					.splineTo(new Vector2d(-30,46), Math.toRadians(212.0))
					.build();

			Trajectory quadTraj4 = drive.trajectoryBuilder(quadTraj3.end(), false)
					.forward(1,
							new MinVelocityConstraint(
									Arrays.asList(
											new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
											new MecanumVelocityConstraint(2.5, DriveConstants.TRACK_WIDTH)
									)
							),
							new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
					.build();

			Trajectory quadTraj5 = drive.trajectoryBuilder(quadTraj4.end(), false)
					.splineTo(new Vector2d(-10,50), Math.toRadians(0.0))
					.splineTo(new Vector2d(32, 54), Math.toRadians(0.0))
					.build();

			Trajectory quadTraj6 = drive.trajectoryBuilder(quadTraj5.end(), false)
					.back(22)
					.build();

		/**Single Ring Trajectory Builders**/

			Trajectory singleTraj1 = drive.trajectoryBuilder(startPose, false)
					.splineToConstantHeading(new Vector2d(-36, 34), Math.toRadians(0.0))
					.build();

			Trajectory singleTraj2 = drive.trajectoryBuilder(singleTraj1.end(), false)
					.forward(18,
							new MinVelocityConstraint(
									Arrays.asList(
											new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
											new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
									)
							),
							new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
					.build();

			Trajectory singleTraj3 = drive.trajectoryBuilder(singleTraj2.end(), false)
					.forward(36)
					.build();

			Trajectory singleTraj3b = drive.trajectoryBuilder(singleTraj3.end().plus(new Pose2d(0,0, Math.toRadians(15.0))), false)
					.back(8)
					.build();

			Trajectory singleTraj4 = drive.trajectoryBuilder(singleTraj3b.end(), false)
					.splineTo(new Vector2d(-26, 38), Math.toRadians(180.0))
					.build();

			Trajectory singleTraj4b = drive.trajectoryBuilder(singleTraj4.end(), false)
					.forward(5,
							new MinVelocityConstraint(
									Arrays.asList(
											new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
											new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
									)
							),
							new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
					.build();

			Trajectory singleTraj5 = drive.trajectoryBuilder(singleTraj4b.end(), false)
					.splineTo(new Vector2d(18, 50), Math.toRadians(0.0))
					.build();

			Trajectory singleTraj6 = drive.trajectoryBuilder(singleTraj5.end(), false)
					.back(20)
					.build();

			Trajectory singleTraj7 = drive.trajectoryBuilder(singleTraj6.end(), false)
					.forward(9)
					.build();

		/**Zero Ring Trajectory Builders**/

			Trajectory zeroTraj1 = drive.trajectoryBuilder(startPose, false)
					.forward(52)
					.addDisplacementMarker(20, () -> {
						robot.launch.setVelocity(3000);
					})
					.build();

			Trajectory zeroTraj2 = drive.trajectoryBuilder(zeroTraj1.end(), false)
					.splineTo(new Vector2d(-8,52), Math.toRadians(45.0))
					.build();

			Trajectory zeroTraj2b = drive.trajectoryBuilder(zeroTraj2.end(), false)
					.back(9,
							new MinVelocityConstraint(
									Arrays.asList(
											new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
											new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
									)
							),
							new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
					.build();

			Trajectory zeroTraj3 = drive.trajectoryBuilder(zeroTraj2b.end(), false)
					.splineTo(new Vector2d(-26,39), Math.toRadians(180.0),
							new MinVelocityConstraint(
									Arrays.asList(
											new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
											new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
									)
							),
							new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
					.build();

			Trajectory zeroTraj3b = drive.trajectoryBuilder(zeroTraj3.end(), false)
					.forward(5,
							new MinVelocityConstraint(
									Arrays.asList(
											new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
											new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
									)
							),
							new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
					.build();

			Trajectory zeroTraj4 = drive.trajectoryBuilder(zeroTraj3.end(), false)
					.splineTo(new Vector2d(-9,56), Math.toRadians(15.0))
					.build();

			Trajectory zeroTraj5 = drive.trajectoryBuilder(zeroTraj4.end(), false)
					.back(18)
					.build();

			Trajectory zeroTraj6 = drive.trajectoryBuilder(zeroTraj5.end(), false)
					.splineToConstantHeading(new Vector2d(7,12), Math.toRadians(0.0))
					.build();

		robot.led3red.setState(true);
		robot.led2red.setState(true);
		robot.led1red.setState(true);
		robot.led3green.setState(false);
		robot.led2green.setState(false);
		robot.led1green.setState(false);

		telemetry.addData("Drive Train", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Angle Adjust", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Tensor Flow Object Detection", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Launcher", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Trajectories", "Initialized");
		telemetry.addData("Status", "Ready");                 // Adds telemetry to the screen to show that the robot is ready
		telemetry.addData("Press Play to Start", "Autonomous");    // Adds telemetry to the screen to tell the drivers that the code is ready to start
		telemetry.update();

		waitForStart();

		if(isStopRequested()) return;

		/*if (step == 0) {
			// getUpdatedRecognitions() will return null if no new information is available since
			// the last time that call was made.
			List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
			if (updatedRecognitions != null) {
				if (updatedRecognitions.size() == 0) {
					ringCount = "None";
					step++;
				}
				// step through the list of recognitions and display boundary info.
				int i = 0;
				for (Recognition recognition : updatedRecognitions) {
					ringCount = recognition.getLabel();
					step++;
				}
			}
		}*/

		if (step == 0) {
			ringCount = "Single";

			step++;
		}

		switch (ringCount) { //Handles changing of program based off of detected rings
			case("Quad"): //Run if four rings are detected
				if (step == 1) {
					robot.led3red.setState(false);
					robot.led2red.setState(false);
					robot.led1red.setState(false);
					robot.led3green.setState(true);
					robot.led2green.setState(true);
					robot.led1green.setState(true);

					drive.followTrajectory(quadTraj1); //Move around the start stack

					step++;
				}

				if (step == 2) {
					armDown(); //Drop Wobble goal into zone
					robot.gripper.setPosition(1); //Open gripper
					sleep(250);
					armUp();

					step++;
				}

				if (step == 3) {
					drive.followTrajectory(quadTraj2); //Back up to launch line
					drive.turn(Math.toRadians(4));
					autoAim(120);
					drive.turn(Math.toRadians(-4));

					step++;
				}

				if (step == 4) {
					drive.followTrajectory(quadTraj3); //Go to second wobble goal
					armDown();

					step++;
				}

				if (step == 5) {
					//drive.followTrajectory(quadTraj4);
					robot.gripper.setPosition(0); //Close the gripper
					sleep(250);
					armUp(); //Grab the wobble goal

					step++;
				}

				if (step == 6) {
					drive.followTrajectory(quadTraj5);
					drive.turn(Math.toRadians(45));

					armDown(); //Drop Wobble goal into zone
					robot.gripper.setPosition(1); //Open gripper
					sleep(250);

					step++;
				}

				if (step == 7) {
					armUp();
					drive.followTrajectory(quadTraj6);

					step++;
				}

				break; //Exit out of switch case

			case("Single"): //Run if one ring is detected
				if (step == 1) {
					robot.led3red.setState(true);
					robot.led2red.setState(false);
					robot.led1red.setState(false);
					robot.led3green.setState(true);
					robot.led2green.setState(true);
					robot.led1green.setState(true);

					//autoAim(415);

					drive.followTrajectory(singleTraj1); //Move in front of start stack
					autoAim(300);

					robot.intakeMotor.setPower(-1); //Start intake motor
					drive.followTrajectory(singleTraj2); //Intake Rings


					step++;
				}

				if (step == 2) {
					drive.followTrajectory(singleTraj3); //Drive to the wobble goal zone
					drive.turn(Math.toRadians(15.0));

					armDown(); //Drop Wobble goal into zone
					robot.gripper.setPosition(1); //Open gripper
					sleep(250);

					robot.intakeMotor.setPower(0); //Stop intake motor

					step++;
				}

				if (step == 3) {
					drive.followTrajectory(singleTraj3b);
					drive.followTrajectory(singleTraj4); //Drive back to the second wobble goal
					drive.followTrajectory(singleTraj4b);

					robot.gripper.setPosition(0); //Close the gripper
					sleep(250);
					armUp(); //Grab the wobble goal

					step++;
				}

				if (step == 4) {
					drive.followTrajectory(singleTraj5); //Drive to the wobble goal zone

					armDown(); //Drop the wobble goal into the zone
					robot.gripper.setPosition(1); //Open the gripper
					sleep(250);
					armUp(); //Pick the arm back up

					step++;
				}

				if (step == 5) {
					drive.followTrajectory(singleTraj6); //Drive back to shoot single ring
					drive.turn(Math.toRadians(-25)); //Turn the robot towards the power shots

					autoAim(500); //Fire last disk at powershots

					drive.followTrajectory(singleTraj7);

					step++;
				}

				break; //Exit out of switch case

			case("None"): //Run if no rings are detected
				if (step == 1) {
					robot.led3red.setState(true);
					robot.led2red.setState(true);
					robot.led1red.setState(false);
					robot.led3green.setState(true);
					robot.led2green.setState(true);
					robot.led1green.setState(true);

					drive.followTrajectory(zeroTraj1); //Move forward to avoid second wobble goal
					drive.turn(Math.toRadians(2)); //Turn a little bit to the left
					autoAim(135); //Fire the three rings
					drive.turn(Math.toRadians(-2)); //Turn a little bit to the right
					drive.followTrajectory(zeroTraj2); //Move in front of wobble goal zone

					armDown(); //Drop the wobble goal into positon
					robot.gripper.setPosition(1); //Open the gripper
					sleep(250);

					step++;
				}

				if (step == 2) {
					drive.followTrajectory(zeroTraj2b); //Back up a little to avoid smacking the wobble goal
					drive.followTrajectory(zeroTraj3); //Move to go pickup the second wobble goal
					sleep(250);
					drive.followTrajectory(zeroTraj3b); //Move forward a tiny bit to grab the wobble goal

					robot.gripper.setPosition(0); //Close gripper
					sleep(250);
					armUp(); //Pickup Wobble Goal

					step++;
				}

				if (step == 3) {
					drive.followTrajectory(zeroTraj4); //Return to wobble goal zone

					armDown(); //Drop the wobble goal into the zone
					robot.gripper.setPosition(1); //Open the gripper
					sleep(250);
					armUp(); //Pickup the arm*/

					step++;
				}

				if (step == 4) {
					drive.followTrajectory(zeroTraj5); //Backup to avoid smacking the wobble goals
					drive.followTrajectory(zeroTraj6); //Move to the line and park/maybe shoot wobble goals

					step++;
				}

				break; //Exit out of switch case

			default:
				telemetry.addData("FATAL ERROR", "Something went wrong :(");
				telemetry.update();
				sleep(10000);
				break;
		}
	}

	/**
	 * Voids for aiming and launching rings
	 */

	public void autoAim(float distance) throws InterruptedException{
		if (distance == 0) {
			return;
		}
		float angle = (float) ((-0.0123*distance)+robot.launchOffset);
		robot.launch.setVelocity(3000);
		sleep(700);
		robot.setLauncherAngle(angle);
		robot.LeftServoPosition = robot.angleAdjustLeft.getPosition();
		robot.RightServoPosition = robot.angleAdjustRight.getPosition();
		robot.launch(4);
		robot.launch.setVelocity(0);
		robot.zeroLauncherAngle();
	}

	public void lockedAim() throws InterruptedException{
		robot.launch.setVelocity(3000);
		sleep(700);

		robot.anglePositionLeft = .295;
		robot.anglePositionRight = .705;
		robot.angleAdjustLeft.setPosition(robot.anglePositionLeft);  	//Set Servo Position
		robot.angleAdjustRight.setPosition(robot.anglePositionRight);   //Set Servo Position

		robot.launch(4);
		robot.LeftServoPosition = robot.angleAdjustLeft.getPosition();
		robot.RightServoPosition = robot.angleAdjustRight.getPosition();
		robot.launch.setVelocity(0);
	}

	public void armDown() {
		robot.armDown = true;
		robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		robot.arm.setTargetPosition(115);
		robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.arm.setPower(1);
		while (robot.arm.isBusy()) {}
		robot.arm.setPower(0);
		robot.gripper.setPosition(1);
		robot.gripperOpen = true;
	}

	public void armUp() {
		robot.arm.setTargetPosition(15);
		robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.arm.setPower(1);
		while (robot.arm.isBusy()) {		}
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

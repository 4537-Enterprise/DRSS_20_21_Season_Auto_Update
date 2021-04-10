package org.firstinspires.ftc.teamcode.Competition_Code;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class InitHardware{

	/* Drivetrain definitions */
		// Drive motor definitions
		public DcMotor motorFrontRight;     // Defines the front right motor
		public DcMotor motorFrontLeft;      // Defines the front left motor
		public DcMotor motorBackRight;      // Defines the back right motor
		public DcMotor motorBackLeft;       // Defines the back left motor

		//Odometer definitions
		public DcMotor leftEncoder;
		public DcMotor rightEncoder;
		public DcMotor centerEncoder;

		//Drive Train Encoder Definitions
		public static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: E8T-360-250-S-D-D-B Encoder
		public static final double     WHEEL_DIAMETER_INCHES   = 1.5 ;     // For figuring circumference
		public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
		public static final double	   COUNTS_PER_DEGREE	   = 0;

		// Mechanum variables definitions
		public double frontRight;          // Sets the double "frontRight"             | Helps with motor calculations
		public double frontLeft;           // Sets the double "frontLeft"              | Helps with motor calculations
		public double backRight;           // Sets the double "backRight"              | Helps with motor calculations
		public double backLeft;            // Sets the double "backLeft                | Helps with motor calculations
		public double speed = 1;           // Sets the double "speed" to one           | Controls overall speed of the drive motor

	/* Payload definitions */
		// Launcher and intake motor definitions
		public DcMotorEx launch;      // Defines the launcher motor
		public DcMotorEx flipper;     // Defines the flipper motor
		public DcMotor intakeMotor; // Defines the intake Motor

		// Angle adjustment servo definitions
		public Servo angleAdjustRight;
		public Servo angleAdjustLeft;

		// Launcher Encoder definition
		public DcMotor boreEncoder;

		// Wobble Goal Arm definitions
		public DcMotor arm;
		public Servo gripper;
		public boolean armDown = true;
		public boolean gripperOpen = false;

		// Angle adjustment variables definitions
		public double anglePositionLeft = .66;
		public double anglePositionRight = .34;
		public double countsPerDegree = 22.75;
		public boolean AutoAimMode = false;
		public double launchOffset = 16.0;
		public double lockedOffset = 0.0;
		public double launcherAngle = 0.0;
		public double LeftServoPosition = 0.0;
		public double RightServoPosition = 0.0;

	/* Vuforia Identification Definitions*/
		public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
		public static final boolean PHONE_IS_PORTRAIT = false;

		public static final String VUFORIA_KEY = "AUz/xEr/////AAABmZjEf3Mc1kGYkOsXVF3u1Gl8gO6qZozGZxty6mcO/xE35elxxgMBwh4/zzwC9Dh4EPKvDexbQAVpjQJzz+Cx+PMYbKiPfvJNsyHDoJkWCPC1skmjKJq/4ctLkD1zGtWPhVUsdGK9ib6ze346j5nHgoFwzoi4SAITZUfQZEj2ccyiWs3zvY2DzbL/QgXrk391epqrpmB6y96vnvCsTUYA6i1y8pg7TZmjUBNWC/3PMr0EHBAFzu+cgtMWVD2sjR9XYcyh9eCRKFNq1aZwikL2P2F4Px5eyujkCVBsnQ0N+dNBo/UCREIF2az5iJY/x+qnrr8aZ2Rj1Gri12gHuKLT7BWS73HKsC9XVURurHz9RmJs";

		public static final float mmPerInch        = 25.4f;
		public static final float mmTargetHeight   = (6) * mmPerInch;

		// Constants for perimeter targets
		public static final float halfField = 72 * mmPerInch;
		public static final float quadField  = 36 * mmPerInch;

		// Class Members
		public VuforiaLocalizer vuforia = null;
		public OpenGLMatrix lastLocation = null;

		public boolean targetVisible = false;
		public float phoneXRotate    = 0;
		public float phoneYRotate    = 0;
		public float phoneZRotate    = 0;

		public String targetName = null;

	/* Tensor Flow Initializations */
		public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
		public static final String LABEL_FIRST_ELEMENT = "Quad";
		public static final String LABEL_SECOND_ELEMENT = "Single";

		public TFObjectDetector tfod;

		public float distanceFromTarget = 0;
		public float x = 0, y = 0;
		public float heading = 0;
		public double expectedAngle = 0;
		public double actualAngle = 0;

		public DigitalChannel led1red;
		public DigitalChannel led1green;
		public DigitalChannel led2red;
		public DigitalChannel led2green;
		public DigitalChannel led3red;
		public DigitalChannel led3green;

	//Local OpMode Members
	HardwareMap hwMap =  null;

	//Constructor
	public InitHardware () {
		//Purposefully Left Empty
	}

	//Initialization Void
	public void init(HardwareMap ahwMap) {
		//Save Reference to Hardware Map
		hwMap = ahwMap;

		//Motor Initialization
		motorFrontRight = hwMap.dcMotor.get("FR");    // Initializes the front right motors name for configuration
		motorFrontLeft = hwMap.dcMotor.get("FL");     // Initializes the front left motors name for configuration
		motorBackRight = hwMap.dcMotor.get("BR");     // Initializes the back right motors name for configuration
		motorBackLeft = hwMap.dcMotor.get("BL");      // Initializes the back left motors name for configuration

		//Motor Direction Initialization
		motorFrontRight.setDirection(DcMotor.Direction.REVERSE);    // Sets the front right motors direction to reverse
		motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);     // Sets the front left motors direction to reverse
		motorBackRight.setDirection(DcMotor.Direction.REVERSE);     // Sets the back right motors direction to reverse
		motorBackLeft.setDirection(DcMotor.Direction.FORWARD);      // Sets the back left motors direction to reverse

		//Motor Brake Definition
		motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		//Odometer Initialization
		leftEncoder = hwMap.get(DcMotor.class, "BL");
		rightEncoder = hwMap.get(DcMotor.class, "FR");
		centerEncoder = hwMap.get(DcMotor.class, "BR");

		//Odometer Direction Initialization
		leftEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
		rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
		centerEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

		//Reset Odometers
		leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		//Reset Odometers
		leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		centerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		//Lift and Intake Motor Initialization
		launch = hwMap.get(DcMotorEx.class, "launch");            // Initializes launcher motor from configuration
		launch.setDirection(DcMotorEx.Direction.FORWARD);  // Sets the launcher motor direction to forward
		launch.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		flipper = hwMap.get(DcMotorEx.class, "flipper");			 // Initializes flipper motor from configuration
		flipper.setDirection(DcMotorEx.Direction.FORWARD); // Sets the flipper motor direction to forward
		intakeMotor = hwMap.get(DcMotor.class, "intakeMotor"); // Initializes intake motor from configuration
		intakeMotor.setDirection(DcMotor.Direction.FORWARD); // Sets the intake motor direction to reverse
		intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		//Angle Adjustment Servo Initialization
		angleAdjustLeft = hwMap.servo.get("angleAdjustL");
		angleAdjustRight = hwMap.servo.get("angleAdjustR");

		//Launcher Encoder Initializations
		boreEncoder = hwMap.get(DcMotor.class, "intakeMotor");
		boreEncoder.setDirection(DcMotorSimple.Direction.FORWARD);

		//Wobble Goal Arm Initialization
		arm = hwMap.get(DcMotor.class, "arm");
		arm.setDirection(DcMotor.Direction.REVERSE);
		arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		gripper = hwMap.servo.get("gripper");

		//LED Initialization
		led1red = hwMap.get(DigitalChannel.class, "led1red");
		led1green = hwMap.get(DigitalChannel.class, "led1green");
		led2red = hwMap.get(DigitalChannel.class, "led2red");
		led2green = hwMap.get(DigitalChannel.class, "led2green");
		led3red = hwMap.get(DigitalChannel.class, "led3red");
		led3green = hwMap.get(DigitalChannel.class, "led3green");

		// change LED mode from input to output
		led1red.setMode(DigitalChannel.Mode.OUTPUT);
		led1green.setMode(DigitalChannel.Mode.OUTPUT);
		led2red.setMode(DigitalChannel.Mode.OUTPUT);
		led2green.setMode(DigitalChannel.Mode.OUTPUT);
		led3red.setMode(DigitalChannel.Mode.OUTPUT);
		led3green.setMode(DigitalChannel.Mode.OUTPUT);
	}

	public void activateTFOD() {
		if (tfod != null) {
			tfod.activate();

			// The TensorFlow software will scale the input images from the camera to a lower resolution.
			// This can result in lower detection accuracy at longer distances (> 55cm or 22").
			// If your target is at distance greater than 50 cm (20") you can adjust the magnification value
			// to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
			// should be set to the value of the images used to create the TensorFlow Object Detection model
			// (typically 16/9).
			tfod.setZoom(2.5, 16.0/9.0);
		}
	}

	public void deactivateTFOD() {
		if (tfod != null) {
			tfod.shutdown();
		}
	}

	public void setLauncherAngle(float angle) {
		for (float i = angle; ((boreEncoder.getCurrentPosition()/countsPerDegree) < (angle/2.5)); i = angle) {
			anglePositionLeft -= 0.01;			//Subtract from current angle on servo
			anglePositionRight += 0.01;			//Add from current angle on servo
			angleAdjustLeft.setPosition(anglePositionLeft);  	//Set Servo Position
			angleAdjustRight.setPosition(anglePositionRight);   //Set Servo Position
		}
		for (float i = angle; ((boreEncoder.getCurrentPosition()/countsPerDegree) < angle); i = angle) {
			anglePositionLeft -= 0.001;			//Subtract from current angle on servo
			anglePositionRight += 0.001;			//Add from current angle on servo
			angleAdjustLeft.setPosition(anglePositionLeft);  	//Set Servo Position
			angleAdjustRight.setPosition(anglePositionRight);   //Set Servo Position
		}
	}

	public void zeroLauncherAngle() {
		anglePositionLeft = .66;
		anglePositionRight = .34;
		angleAdjustLeft.setPosition(anglePositionLeft);  	//Set Servo Position
		angleAdjustRight.setPosition(anglePositionRight);   //Set Servo Position
	}

	public void launch(int rings) throws InterruptedException{
		for (int i = 1; i <= rings; i++) {
			flipper.setTargetPosition(flipper.getCurrentPosition() + 20);
			flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			flipper.setVelocity(600);
			while (flipper.isBusy()) {

			}
			flipper.setVelocity(-600);
			sleep(100);
		}
		flipper.setVelocity(0);
	}

	public void stopMotors() {
		motorFrontRight.setPower(0);   // Sets the front right motors speed to the previous double
		motorFrontLeft.setPower(0);     // Sets the front left motors speed to the previous double
		motorBackRight.setPower(0);     // Sets the back right motors speed to the previous double
		motorBackLeft.setPower(0);       // Sets the back left motors speed to the previous double
	}
}


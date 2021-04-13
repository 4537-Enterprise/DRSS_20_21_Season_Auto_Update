package org.firstinspires.ftc.teamcode.Competition_Code;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class InitHardwareRoadRunner{

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
	public InitHardwareRoadRunner() {
		//Purposefully Left Empty
	}

	//Initialization Void
	public void init(HardwareMap ahwMap) {
		//Save Reference to Hardware Map
		hwMap = ahwMap;

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

	/** Auto Aim Voids so I can kill the program **/
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
}


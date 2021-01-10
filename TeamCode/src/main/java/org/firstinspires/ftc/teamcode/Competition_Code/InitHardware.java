package org.firstinspires.ftc.teamcode.Competition_Code;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

import java.util.ArrayList;
import java.util.List;

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

		// Mechanum variables definitions
		public double frontRight;          // Sets the double "frontRight"             | Helps with motor calculations
		public double frontLeft;           // Sets the double "frontLeft"              | Helps with motor calculations
		public double backRight;           // Sets the double "backRight"              | Helps with motor calculations
		public double backLeft;            // Sets the double "backLeft                | Helps with motor calculations
		public double speed = 1;           // Sets the double "speed" to one           | Controls overall speed of the drive motor

	/* Payload definitions */
		// Launcher and intake motor definitions
		public DcMotor intake;      // Defines the intake motor
		public DcMotor launch;      // Defines the launcher motor
		public DcMotor flipper;     // Defines the flipper motor

		// Angle adjustment servo definitions
		public Servo angleAdjustRight;
		public Servo angleAdjustLeft;

		// Launcher Encoder definition
		public DcMotor boreEncoder;

		// Wobble Goal Arm definitions
		public DcMotor arm;

		// Angle adjustment variables definitions
		public double anglePositionLeft = .66;
		public double anglePositionRight = .34;

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

		public WebcamName webcamName = null;

		public boolean targetVisible = false;
		public float phoneXRotate    = 0;
		public float phoneYRotate    = 0;
		public float phoneZRotate    = 0;

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
		motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);     // Sets the front left motors direction to reverse
		motorBackRight.setDirection(DcMotor.Direction.REVERSE);     // Sets the back right motors direction to reverse
		motorBackLeft.setDirection(DcMotor.Direction.REVERSE);      // Sets the back left motors direction to reverse

		//Odometer Initialization
		leftEncoder = hwMap.get(DcMotor.class, "FL");
		rightEncoder = hwMap.get(DcMotor.class, "FR");
		centerEncoder = hwMap.get(DcMotor.class, "BL");

		//Odometer Direction Initialization
		leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
		rightEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
		centerEncoder.setDirection(DcMotorSimple.Direction.FORWARD);

		//Lift and Intake Motor Initialization
		intake = hwMap.dcMotor.get("intake");            // Initializes the intake motor from configuration
		intake.setDirection(DcMotor.Direction.REVERSE);  // Sets the intake motor direction to forward
		launch = hwMap.dcMotor.get("launch");            // Initializes launcher motor from configuration
		launch.setDirection(DcMotor.Direction.FORWARD);  // Sets the launcher motor direction to forward
		flipper = hwMap.get(DcMotorEx.class, "flipper");			 // Initializes flipper motor from configuration
		flipper.setDirection(DcMotor.Direction.REVERSE); // Sets the flipper motor direction to forward

		//Angle Adjustment Servo Initialization
		angleAdjustLeft = hwMap.servo.get("angleAdjustL");
		angleAdjustRight = hwMap.servo.get("angleAdjustR");

		//Launcher Encoder Initializations
		boreEncoder = hwMap.get(DcMotor.class, "intake");
		boreEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

		//Launcher Encoder Direction Initialization
		boreEncoder.setDirection(DcMotorSimple.Direction.FORWARD);

		//Wobble Goal Arm Initialization
		arm = hwMap.get(DcMotor.class, "arm");
		arm.setDirection(DcMotor.Direction.REVERSE);
		arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		//Vuforia Initializations
		webcamName = hwMap.get(WebcamName.class, "Webcam 1");

		int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

		parameters.vuforiaLicenseKey = VUFORIA_KEY;

		parameters.cameraName = webcamName;

		// Make sure extended tracking is disabled for this example.
		parameters.useExtendedTracking = false;

		//  Instantiate the Vuforia engine
		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		// Load the data sets for the trackable objects. These particular data
		// sets are stored in the 'assets' part of our application.
		VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
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

		//Set the position of the perimeter targets with relation to origin (center of field)
		redAllianceTarget.setLocation(OpenGLMatrix
				.translation(0, -halfField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

		blueAllianceTarget.setLocation(OpenGLMatrix
				.translation(0, halfField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
		frontWallTarget.setLocation(OpenGLMatrix
				.translation(-halfField, 0, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

		// The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
		blueTowerGoalTarget.setLocation(OpenGLMatrix
				.translation(halfField, quadField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
		redTowerGoalTarget.setLocation(OpenGLMatrix
				.translation(halfField, -quadField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

		// We need to rotate the camera around it's long axis to bring the correct camera forward.
		if (CAMERA_CHOICE == BACK) {
			phoneYRotate = -90;
		} else {
			phoneYRotate = 90;
		}

		// Rotate the phone vertical about the X axis if it's in portrait mode
		if (PHONE_IS_PORTRAIT) {
			phoneXRotate = 90 ;
		}

		// Next, translate the camera lens to where it is on the robot.
		// In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
		final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
		final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
		final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

		OpenGLMatrix robotFromCamera = OpenGLMatrix
				.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

		/**  Let all the trackable listeners know where the phone is.  */
		for (VuforiaTrackable trackable : allTrackables) {
			((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
		}
	}
}


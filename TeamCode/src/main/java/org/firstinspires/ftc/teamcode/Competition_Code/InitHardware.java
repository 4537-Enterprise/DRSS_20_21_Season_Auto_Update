package org.firstinspires.ftc.teamcode.Competition_Code;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

		// Angle adjustment variables definitions
		public double anglePositionLeft = .5;
		public double anglePositionRight = .5;

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
		intake.setDirection(DcMotor.Direction.FORWARD);  // Sets the intake motor direction to forward
		launch = hwMap.dcMotor.get("launch");            // Initializes launcher motor from configuration
		launch.setDirection(DcMotor.Direction.FORWARD);  // Sets the launcher motor direction to forward
		flipper = hwMap.get(DcMotorEx.class, "flipper");			 // Initializes flipper motor from configuration
		flipper.setDirection(DcMotor.Direction.FORWARD); // Sets the flipper motor direction to forward

		//Angle Adjustment Servo Initialization
		angleAdjustLeft = hwMap.servo.get("angleAdjustL");
		angleAdjustRight = hwMap.servo.get("angleAdjustR");

		//Launcher Encoder Initializations
		boreEncoder = hwMap.get(DcMotor.class, "launch");
		flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		//Launcher Encoder Direction Initialization
		boreEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
	}
}


package org.firstinspires.ftc.teamcode.Test_Code;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Turn Calibration", group = "Auto")
public class encoderTurnCalibration extends LinearOpMode{

	// Drive motor definitions
	public DcMotor motorFrontRight;     // Defines the front right motor
	public DcMotor motorFrontLeft;      // Defines the front left motor
	public DcMotor motorBackRight;      // Defines the back right motor
	public DcMotor motorBackLeft;       // Defines the back left motor

	//Odometer definitions
	public DcMotor leftEncoder;
	public DcMotor rightEncoder;
	public DcMotor centerEncoder;

	//Gyro Definitions
	// Defines the gyro
	BNO055IMU imu;

	// State used for updating telemetry
	Orientation angles;
	Acceleration gravity;

	//Gyro Turning Definitions
	static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
	static final double     P_TURN_COEFF            = 0.15;     // Larger is more responsive, but also less stable

	private int step = 1;

	//Encoder Reading Variables
	double leftEncValue = 0;
	double rightEncValue = 0;
	double motorEncValuesAvg = 0;

	double encReading10 = 0;
	double encReading15 = 0;
	double encReading30 = 0;
	double encReading45 = 0;
	double encReading90 = 0;
	double encReading90Avg = 0;
	double calculatedEncReading = 0;

	@Override
	public void runOpMode() throws InterruptedException{

		//Motor Initialization
		motorFrontRight = hardwareMap.dcMotor.get("FR");    // Initializes the front right motors name for configuration
		motorFrontLeft = hardwareMap.dcMotor.get("FL");     // Initializes the front left motors name for configuration
		motorBackRight = hardwareMap.dcMotor.get("BR");     // Initializes the back right motors name for configuration
		motorBackLeft = hardwareMap.dcMotor.get("BL");      // Initializes the back left motors name for configuration

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
		leftEncoder = hardwareMap.get(DcMotor.class, "FL");
		rightEncoder = hardwareMap.get(DcMotor.class, "FR");
		centerEncoder = hardwareMap.get(DcMotor.class, "BL");

		//Odometer Direction Initialization
		leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
		rightEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
		centerEncoder.setDirection(DcMotorSimple.Direction.FORWARD);

		//Gyro Initialization

		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

		parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode

		parameters.loggingEnabled = true;

		parameters.loggingTag = "IMU";

		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

		imu = hardwareMap.get(BNO055IMU.class, "imu");

		imu.initialize(parameters);

		waitForStart();

		if (step == 1) { //Stop and Reset Encoders
				stopAndResetEncoder();
				step++;
			}

		if (step == 2) { //Turn 10 Degrees
			encTelemetry();

			gyroTurn(.1,10);
			gyroHold(.1,10,1);

			leftEncValue = leftEncoder.getCurrentPosition();
			rightEncValue = rightEncoder.getCurrentPosition();

			motorEncValuesAvg = (Math.abs(rightEncValue) + Math.abs(leftEncValue)) / 2;
			encReading10 = motorEncValuesAvg * 9;

			stopAndResetEncoder();

			step++;
		}

		if (step == 3) { //Turn 15 Degrees
			encTelemetry();

			gyroTurn(.1,25);
			gyroHold(.1,25,1);

			leftEncValue = leftEncoder.getCurrentPosition();
			rightEncValue = rightEncoder.getCurrentPosition();

			motorEncValuesAvg = (Math.abs(rightEncValue) + Math.abs(leftEncValue)) / 2;
			encReading15 = motorEncValuesAvg * 6;

			stopAndResetEncoder();

			step++;
		}

		if (step == 4) { //Turn 30 Degrees
			encTelemetry();

			gyroTurn(.1,55);
			gyroHold(.1,55,1);

			leftEncValue = leftEncoder.getCurrentPosition();
			rightEncValue = rightEncoder.getCurrentPosition();

			motorEncValuesAvg = (Math.abs(rightEncValue) + Math.abs(leftEncValue)) / 2;
			encReading30 = motorEncValuesAvg * 3;

			stopAndResetEncoder();

			step++;
		}

		if (step == 5) { //Turn 45 Degrees
			encTelemetry();

			gyroTurn(.1,100);
			gyroHold(.1,100,1);

			leftEncValue = leftEncoder.getCurrentPosition();
			rightEncValue = rightEncoder.getCurrentPosition();

			motorEncValuesAvg = (Math.abs(rightEncValue) + Math.abs(leftEncValue)) / 2;
			encReading45 = motorEncValuesAvg * 2;

			stopAndResetEncoder();

			step++;
		}

		if (step == 6) { //Turn back to 0 Degrees
			encTelemetry();

			gyroTurn(.1,0);
			gyroHold(.1,0,1);

			stopAndResetEncoder();

			step++;
		}

		if (step == 7) { //Turn 90 Degrees
			encTelemetry();

			gyroTurn(.1,90);
			gyroHold(.1,90,1);

			leftEncValue = leftEncoder.getCurrentPosition();
			rightEncValue = rightEncoder.getCurrentPosition();

			motorEncValuesAvg = (Math.abs(rightEncValue) + Math.abs(leftEncValue)) / 2;
			encReading90 = motorEncValuesAvg;

			stopAndResetEncoder();

			step++;
		}

		if (step == 8) { //Calculate Average Encoder Value
			encReading90Avg = (encReading10 + encReading15 + encReading30 + encReading45 + encReading90) / 5;
			calculatedEncReading = encReading90Avg / 90;

			telemetry.addData("10 Degrees", encReading10);
			telemetry.addData("15 Degrees", encReading15);
			telemetry.addData("30 Degrees", encReading30);
			telemetry.addData("45 Degrees", encReading45);
			telemetry.addData("90 Degrees", encReading90);
			telemetry.addData("90 Degree Avg Enc Value", encReading90Avg);
			telemetry.addData("1 Degree Avg Enc Value", calculatedEncReading);
			telemetry.update();
		}


	}

	public void gyroTurn (  double speed, double angle) {

		// keep looping while we are still active, and not on heading.
		while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
			// Update telemetry & Allow time for other processes to run.
			telemetry.update();
		}
	}

	/**
	 *  Method to obtain & hold a heading for a finite amount of time
	 *  Move will stop once the requested time has elapsed
	 *
	 * @param speed      Desired speed of turn.
	 * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
	 *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
	 *                   If a relative angle is required, add/subtract from current heading.
	 * @param holdTime   Length of time (in seconds) to hold the specified heading.
	 */
	public void gyroHold( double speed, double angle, double holdTime) {

		ElapsedTime holdTimer = new ElapsedTime();

		// keep looping while we have time remaining.
		holdTimer.reset();
		while (opModeIsActive() && (holdTimer.time() < holdTime)) {
			// Update telemetry & Allow time for other processes to run.
			onHeading(speed, angle, P_TURN_COEFF);
			telemetry.update();
		}

		// Stop all motion;
		motorFrontLeft.setPower(0);
		motorBackLeft.setPower(0);
		motorFrontRight.setPower(0);
		motorBackRight.setPower(0);
	}

	/**
	 * Perform one cycle of closed loop heading control.
	 *
	 * @param speed     Desired speed of turn.
	 * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
	 *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
	 *                  If a relative angle is required, add/subtract from current heading.
	 * @param PCoeff    Proportional Gain coefficient
	 * @return
	 */
	boolean onHeading(double speed, double angle, double PCoeff) {
		double   error ;
		double   steer ;
		boolean  onTarget = false ;
		double leftSpeed;
		double rightSpeed;

		// determine turn power based on +/- error
		error = getError(angle);

		if (Math.abs(error) <= HEADING_THRESHOLD) {
			steer = 0.0;
			leftSpeed  = 0.0;
			rightSpeed = 0.0;
			onTarget = true;
		}
		else {
			steer = getSteer(error, PCoeff);
			rightSpeed  = speed * steer;
			leftSpeed   = -rightSpeed;
		}

		// Send desired speeds to motors.
		motorFrontLeft.setPower(leftSpeed);
		motorBackLeft.setPower(leftSpeed);
		motorFrontRight.setPower(rightSpeed);
		motorBackRight.setPower(rightSpeed);

		// Display it for the driver.
		telemetry.addData("Target", "%5.2f", angle);
		telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
		telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

		return onTarget;
	}

	/**
	 * getError determines the error between the target angle and the robot's current heading
	 * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
	 * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
	 *          +ve error means the robot should turn LEFT (CCW) to reduce error.
	 */
	public double getError(double targetAngle) {

		double robotError;

		// calculate error in -179 to +180 range  (
		robotError = targetAngle - angles.firstAngle;
		while (robotError > 180)  robotError -= 360;
		while (robotError <= -180) robotError += 360;
		return robotError;
	}

	/**
	 * returns desired steering force.  +/- 1 range.  +ve = steer left
	 * @param error   Error angle in robot relative degrees
	 * @param PCoeff  Proportional Gain Coefficient
	 * @return
	 */
	public double getSteer(double error, double PCoeff) {
		return Range.clip(error * PCoeff, -1, 1);
	}

	public void stopAndResetEncoder() {
		//Stop and Reset Encoders
		motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		telemetry.addData("Reseting", " Encoders");
		telemetry.update();

		//Run Using Encoders
		motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void encTelemetry() {
		telemetry.addData("10 Degrees", encReading10);
		telemetry.addData("15 Degrees", encReading15);
		telemetry.addData("30 Degrees", encReading30);
		telemetry.addData("45 Degrees", encReading45);
		telemetry.addData("90 Degrees", encReading90);
		telemetry.update();
	}
}

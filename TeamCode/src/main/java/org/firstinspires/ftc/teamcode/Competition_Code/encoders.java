package org.firstinspires.ftc.teamcode.Competition_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "encoder test", group = "test")
public class encoders extends LinearOpMode{

	private InitHardware robot = new InitHardware();  //Load hardware from hardware map

	int newLeftTarget;
	int newRightTarget;
	int newCenterTarget;

	int step = 1;



	@Override
	public void runOpMode() throws InterruptedException{
		robot.init(hardwareMap); //Initialize hardware

		telemetry.addData("REady", "");
		telemetry.update();

		waitForStart();

		if (step == 1) {
			drive(.5,12,1);
			step++;
		}
	}

	public void drive (double speed, double Inches, int directionModifier) {

		// Ensure that the opmode is still active
		if (opModeIsActive()){

			// Math to calculate each target position for the motors
			newLeftTarget = robot.leftEncoder.getCurrentPosition() + (int) (Inches * robot.COUNTS_PER_INCH);
			newRightTarget = robot.rightEncoder.getCurrentPosition() + (int) (Inches * robot.COUNTS_PER_INCH);

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
}

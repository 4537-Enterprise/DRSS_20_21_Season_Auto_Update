package org.firstinspires.ftc.teamcode.Competition_Code.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Competition_Code.InitHardware;

@TeleOp(name = "Scorpion", group = "TeleOp")
public class Scorpion extends LinearOpMode{

	private InitHardware robot = new InitHardware();  //Load hardware from hardware map

	@Override
	public void runOpMode() throws InterruptedException{

		robot.init(hardwareMap); //Initialize hardware

		robot.angleAdjustLeft.setPosition(.5); 	//Start servos at lowest point
		robot.angleAdjustRight.setPosition(.5); //Start servos at lowest point
		robot.launch.setPower(.5);				//Start up launcher

		telemetry.addData("Drive Train: ", "Initialized");      // Adds telemetry to the screen to show that the drive train is initialized
		telemetry.addData("Payload: ", "Initialized");          // Adds telemetry to the screen to show that the payload is initialized
		telemetry.addData("Status: ", "Ready");                 // Adds telemetry to the screen to show that the robot is ready
		telemetry.addData("Press Play to Start ", "TeleOp");    // Adds telemetry to the screen to tell the drivers that the code is ready to start
		telemetry.update();

		waitForStart();

		while (opModeIsActive()){

			/**Mechanum drive controls**/
				// left stick controls direction
				// right stick X controls rotation
				float gamepad1LeftY = gamepad1.left_stick_y;        // Sets the gamepads left sticks y position to a float so that we can easily track the stick
				float gamepad1LeftX = -gamepad1.left_stick_x;       // Sets the gamepads left sticks x position to a float so that we can easily track the stick
				float gamepad1RightX = -gamepad1.right_stick_x;     // Sets the gamepads right sticks x position to a float so that we can easily track the stick

				// Mechanum formulas
				double FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;     // Combines the inputs of the sticks to clip their output to a value between 1 and -1
				double FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;     // Combines the inputs of the sticks to clip their output to a value between 1 and -1
				double BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;      // Combines the inputs of the sticks to clip their output to a value between 1 and -1
				double BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;      // Combines the inputs of the sticks to clip their output to a value between 1 and -1

				// sets speed
				robot.frontRight = Range.clip(Math.pow(FrontRight, 3), -robot.speed, robot.speed);    // Slows down the motor and sets its max/min speed to the double "speed"
				robot.frontLeft = Range.clip(Math.pow(FrontLeft, 3), -robot.speed, robot.speed);      // Slows down the motor and sets its max/min speed to the double "speed"
				robot.backRight = Range.clip(Math.pow(BackRight, 3), -robot.speed, robot.speed);      // Slows down the motor and sets its max/min speed to the double "speed"
				robot.backLeft = Range.clip(Math.pow(BackLeft, 3), -robot.speed, robot.speed);        // Slows down the motor and sets its max/min speed to the double "speed"

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
				if (gamepad1.a) {
					robot.intake.setPower(1); //Start running intake motor
				}
				else {
					robot.intake.setPower(0); //Stop running intake motor
				}
			/**End of intake controls**/

			/**Launcher Controls**/
				if (gamepad1.dpad_up) { 										//Move the launcher up
					robot.anglePositionLeft = robot.anglePositionLeft - 0.01;  	//Subtract from current angle on servo
					robot.anglePositionRight = robot.anglePositionRight + 0.01;	//Add from current angle on servo
				}

				if (gamepad1.dpad_down) { 										//Move the launcher down
					robot.anglePositionLeft = robot.anglePositionLeft + 0.01;  	//Add from current angle on servo
					robot.anglePositionRight = robot.anglePositionRight - 0.01;	//Subtract from current angle on servo
				}

				if (gamepad1.y) { 					//Put the launcher at its lowest point
					robot.anglePositionLeft = .5;	//Set servo positions to .5
					robot.anglePositionRight = .5;	//Set servo positions to .5
				}

				robot.angleAdjustLeft.setPosition(robot.anglePositionLeft);  	//Set Servo Position
				robot.angleAdjustRight.setPosition(robot.anglePositionRight);   //Set Servo Position

				if (gamepad1.x) {				//Ramp up launcher
					 robot.launch.setPower(1);  //Set launcher motor to full speed
				}
				else {							//Slow down launcher when not shooting
					robot.launch.setPower(.5);	//Set launcher motor to half speed
				}


			/**End of launcher controls**/

			/**Telemetry**/
				telemetry.addData("Speed: ", robot.speed);
				telemetry.update();
			/**End of telemetry**/
		}
	}
}

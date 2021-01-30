package org.firstinspires.ftc.teamcode.Competition_Code.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Competition_Code.InitHardware;
import org.firstinspires.ftc.teamcode.Competition_Code.encoders;

@Autonomous(name = "ScorpionAuto", group = "Auto")
public class ScorpionAuto extends LinearOpMode{

	private InitHardware robot = new InitHardware();  	//Load hardware from hardware map
	private encoders encoder = new encoders();			//Load encoder codes
	private int step = 1;

	@Override
	public void runOpMode() throws InterruptedException{

		robot.init(hardwareMap); //Initialize hardware

		waitForStart();

		if (step == 1) {
			encoder.drive(1,12,1);
			step++;
		}

	}
}

package org.firstinspires.ftc.teamcode.Test_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static java.lang.Thread.sleep;

@Disabled
@Autonomous(name = "FlippyTest", group = "Test")
public class BareMotorTest extends LinearOpMode{

	private DcMotorEx flipper;

	private int step = 1;

	@Override
	public void runOpMode() throws InterruptedException{
		flipper = hardwareMap.get(DcMotorEx.class, "flippy");
		flipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		telemetry.addData("Ready", ":)");
		telemetry.update();

		waitForStart();

		if (step == 1) {
			launch(1);
			step++;
		}
	}

	public void launch(int rings) throws InterruptedException{
		for (int i = 1; i <= rings; i++) {
			flipper.setTargetPosition(100);
			flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			flipper.setPower(1);
			while (flipper.isBusy()) {

			}
			flipper.setTargetPosition(-5);
			while (flipper.isBusy()) {

			}
			flipper.setPower(0);
			sleep(400);
		}
	}
}

package org.firstinspires.ftc.teamcode.Test_Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Odometer Troublshooting", group = "test")
public class OdometerTest extends LinearOpMode{

	DcMotor encoder;

	@Override
	public void runOpMode() throws InterruptedException{

		encoder = hardwareMap.get(DcMotor.class, "BR");

		waitForStart();

		while (opModeIsActive()) {
			telemetry.addData("Position:", encoder.getCurrentPosition());
			telemetry.update();
		}

	}
}

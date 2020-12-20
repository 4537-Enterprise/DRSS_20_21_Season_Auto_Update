package org.firstinspires.ftc.teamcode.Test_Code.Previous_Code.Auto_Tests.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "OdometryTest", group = "Autonomous")
public class OdometryTest extends LinearOpMode{

	private RunOdometry OD = new RunOdometry();
	private double step = 1;

	@Override
	public void runOpMode() throws InterruptedException{
		OD.init(hardwareMap);

		waitForStart();

		/*while (opModeIsActive()) {
			telemetry.addData("Left Encoder: ", OD.getLeftTicks());
			telemetry.addData("Right Encoder: ", OD.getRightTicks());
			telemetry.addData("Center Encoder: ", OD.getCenterTicks());
			telemetry.update();
		}*/

		if (step == 1) {
			OD.runToPosition(10,0,1);
			step++;
		}

		if (step == 2) {
			OD.setMotorPower(0, 0, 0, 0);
			step++;
		}

		while (step == 3 && opModeIsActive()) {
			telemetry.addData("Front Right: ", OD.FRPOWER);
			telemetry.addData("Front Left: ", OD.FLPOWER);
			telemetry.addData("Back Right: ", OD.BRPOWER);
			telemetry.addData("Back Left: ", OD.BLPOWER);
			telemetry.update();
		}
	}
}

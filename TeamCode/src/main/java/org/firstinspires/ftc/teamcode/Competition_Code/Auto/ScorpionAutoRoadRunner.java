package org.firstinspires.ftc.teamcode.Competition_Code.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Competition_Code.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "ScorpionAutoRoadRunner", group = "Auto")
public class ScorpionAutoRoadRunner extends LinearOpMode{

	int step = 1;

	@Override
	public void runOpMode() throws InterruptedException{

		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

		// We want to start the bot at x: -63, y: 18, heading: 0 degrees
		Pose2d startPose = new Pose2d(-63, 18, Math.toRadians(0.0));
		drive.setPoseEstimate(startPose);

		Trajectory traj1 = drive.trajectoryBuilder(startPose)
				.forward(12)
				.build();

		Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
				.splineToLinearHeading(new Pose2d(60, 40, Math.toRadians(90.0)), Math.toRadians(0.0))
				.build();

		waitForStart();

		if(isStopRequested()) return;

		if (step == 1) {
			drive.followTrajectory(traj1);

			step++;
		}

		if (step == 2) {
			drive.followTrajectory(traj2);

			step++;
		}

	}
}

package org.firstinspires.ftc.teamcode.Competition_Code.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper;
import org.firstinspires.ftc.teamcode.Competition_Code.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Competition_Code.RoadRunner.drive.SampleMecanumDrive;

import java.util.Arrays;

@Autonomous(name = "ScorpionAutoRoadRunner", group = "Auto")
public class ScorpionAutoRoadRunner extends LinearOpMode{

	int step = 1;

	@Override
	public void runOpMode() throws InterruptedException{

		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

		// We want to start the bot at x: -63, y: 18, heading: 0 degrees
		Pose2d startPose = new Pose2d(-63, 32, Math.toRadians(0.0));
		drive.setPoseEstimate(startPose);

		Trajectory traj1 = drive.trajectoryBuilder(startPose, false)
				.splineToConstantHeading(new Vector2d(-6, 10), Math.toRadians(0.0))
				.build();

		Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), false)
				.splineTo(new Vector2d(50,42), Math.toRadians(90.0))
				.build();

		Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
				.splineTo(new Vector2d(-2, 32), Math.toRadians(180.0))
				.build();

		Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0,0,Math.toRadians(180.0))), false)
				.splineTo(new Vector2d(-32, 32), Math.toRadians(180.0),
						new MinVelocityConstraint(
								Arrays.asList(
										new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
										new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
								)
						),
						new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
				.build();

		Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), true)
				.splineTo(new Vector2d(-24, 32), Math.toRadians(0.0))
				.build();

		Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
				.splineTo(new Vector2d(-32,32), Math.toRadians(160.0))
				.build();

		waitForStart();

		if(isStopRequested()) return;

		if (step == 1) { //Maneuver around the start stack
			drive.followTrajectory(traj1);

			step++;
		}

		if (step == 2) { //Go to wobble goal target area
			drive.followTrajectory(traj2);

			step++;
		}

		if (step == 3) { //Back up to launch line
			drive.followTrajectory(traj3);
			sleep(100);

			step++;
		}

		if (step == 4) { //Turn around to pickup rings
			drive.turn(Math.toRadians(180.0));

			step++;
		}

		if (step == 5) { //Drive to pickup rings
			drive.followTrajectory(traj4);

			step++;
		}

		if (step == 6) { //Backup from rings and turn 180 degrees
			drive.followTrajectory(traj5);
			drive.turn(Math.toRadians(180));
			sleep(1000);

			step++;
		}

		if (step == 7) { //Turn back to face with wobble goal
			drive.turn(Math.toRadians(160));

			step++;
		}

		/*if (step == 8) { //Drive to wobble goal
			drive.followTrajectory(traj6);
		}*/

	}
}

package org.firstinspires.ftc.teamcode.Test_Code.Previous_Code.Auto_Tests.Pseudodometry;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Test_Code.Previous_Code.AutoHardwareGalileo;

public class coordinateSystemClasses{

	AutoHardwareGalileo robot = new AutoHardwareGalileo();   //Calls Upon Robot Definitions File

	private ElapsedTime runtime = new ElapsedTime(); //Sets timer for encoders

	double step = 1; //Sets the steps for the autonomous

	//Odometry Variables
	double currentAngle = 0;
	double currentPositionX = 0;
	double currentPositionY = 0;


}

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.controls.DriverControls;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {
	//#region robot
	@Override
	public void robotInit() {
		Drivetrain.setDimensions(30, 30);	//TODO get actual measurements before testing
	}

	@Override
	public void robotPeriodic() {

	}
	//#endregion

	//#region autonomous
	@Override
	public void autonomousInit() {
		
	}

	@Override
	public void autonomousPeriodic() {
		
	}
	//#endregion

	//#region teleop
	@Override
	public void teleopInit() {
		
	}

	@Override
	public void teleopPeriodic() {
		DriverControls.driverControls();
	}
	//#endregion

	//#region test
	@Override
	public void testInit() {
		
	}

	@Override
	public void testPeriodic() {

	}
	//#endregion

	//#region disabled
	@Override
	public void disabledInit() {
		
	}

	@Override
	public void disabledPeriodic() {
		Drivetrain.drive(0, 0, 0);
	}
	//#endregion
}
package frc.robot.controls;

import frc.robot.subsystems.Drivetrain;

public class DriverControls extends PS4Controller {
	private static DriverControls singletonInstance = new DriverControls(0);

	/**
	 * calls controls method of singleton DriverControls instance
	 */
	public static void driverControls() {
		singletonInstance.controls();
	}

	//#region singleton instance methods
	/**
	 * initializes driver controller
	 * @param port port of the driver controller
	 */
	private DriverControls(int port) {
		super(port);
	}

	/**
	 * performs actions based on driver controller input
	 */
	private void controls() {
		double leftXAxis;
		double leftYAxis;
		double rightXAxis;

		//TODO change for correct pos/neg values
		leftXAxis = Math.abs(getLeftXAxis()) < 0.1 ? 0 : getLeftXAxis();
		leftYAxis = Math.abs(getLeftYAxis()) < 0.1 ? 0 : getLeftYAxis();
		rightXAxis = Math.abs(getRightXAxis()) < 0.1 ? 0 : getRightXAxis();
		
		Drivetrain.drive(leftXAxis, leftYAxis, rightXAxis);
	}
	//#endregion
}
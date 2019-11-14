package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;

public class PS4Controller {
	private final Joystick joy;

	/**
	 * initializes a PS4 controller
	 * @param port port of the controller
	 */
	public PS4Controller(int port) {
		joy = new Joystick(port);
	}

	// #region buttons
	/**
	 * @return value of triangle button
	 */
	public boolean getTriButton() {
		return joy.getRawButton(4);
	}

	/**
	 * @return value of circle button
	 */
	public boolean getCircleButton() {
		return joy.getRawButton(3);
	}

	/**
	 * @return value of X button
	 */
	public boolean getXButton() {
		return joy.getRawButton(2);
	}

	/**
	 * @return value of square button
	 */
	public boolean getSquareButton() {
		return joy.getRawButton(1);
	}

	/**
	 * @return value of left stick button
	 */
	public boolean getLeftStickButton() {
		return joy.getRawButton(11);
	}

	/**
	 * @return value of right stick button
	 */
	public boolean getRightStickButton() {
		return joy.getRawButton(12);
	}

	/**
	 * @return value of left bumper button
	 */
	public boolean getLeftBumperButton() {
		return joy.getRawButton(5);
	}

	/**
	 * @return value of right bumper button
	 */
	public boolean getRightBumperButton() {
		return joy.getRawButton(6);
	}

	/**
	 * @return value of the share button
	 */
	public boolean getShareButton() {
		return joy.getRawButton(9);
	}

	/**
	 * @return value of the options button
	 */
	public boolean getOptionsButton() {
		return joy.getRawButton(10);
	}

	/**
	 * @return value of the play station button
	 */
	public boolean getPSButton() {
		return joy.getRawButton(13);
	}

	/**
	 * @return value of the trackpad button
	 */
	public boolean getTrackpadButton() {
		return joy.getRawButton(14);
	}

	/**
	 * @return value of the y axis of the left stick (-1.0 to 1.0)
	 */
	public double getLeftYAxis() {
		return joy.getRawAxis(1);
	}

	/**
	 * @return value of the x axis of the left stick (-1.0 to 1.0)
	 */
	public double getLeftXAxis() {
		return joy.getRawAxis(0);
	}

	/**
	 * @return value of the y axis of the right stick (-1.0 to 1.0)
	 */
	public double getRightYAxis() {
		return joy.getRawAxis(5);
	}

	/**
	 * @return value of the x axis of the right stick (-1.0 to 1.0)
	 */
	public double getRightXAxis() {
		return joy.getRawAxis(2);
	}

	/**
	 * @return value of the left trigger axis (0.0 to 1.0)
	 */
	public double getLeftTriggerAxis() {
		return (joy.getRawAxis(3) + 1) / 2;
	}

	/**
	 * @return value of the right trigger axis (0.0 to 1.0)
	 */
	public double getRightTriggerAxis() {
		return (joy.getRawAxis(4) + 1) / 2;
	}

	/**
	 * @return value of the up d-pad button
	 */
	public boolean getButtonDUp() {
		return joy.getPOV() == 0;
	}

	/**
	 * @return value of the right d-pad button
	 */
	public boolean getButtonDRight() {
		return joy.getPOV() == 90;
	}

	/**
	 * @return value of the down d-pad button
	 */
	public boolean getButtonDDown() {
		return joy.getPOV() == 180;
	}

	/**
	 * @return value of the left d-pad button
	 */
	public boolean getButtonDLeft() {
		return joy.getPOV() == 270;
	}

	/**
	 * @return true if any button/axis is being used
	 */
	public boolean anythingPressed() {
		return getTriButton() || getSquareButton() || getCircleButton() || getXButton() ||

				getPSButton() || getShareButton() || getOptionsButton() ||

				getButtonDDown() || getButtonDLeft() || getButtonDRight() || getButtonDUp() ||

				getLeftBumperButton() || Math.abs(getLeftTriggerAxis()) > .1 ||

				getRightBumperButton() || Math.abs(getRightTriggerAxis()) > .1 ||

				getLeftStickButton() || Math.abs(getLeftXAxis()) > .1 || Math.abs(getLeftYAxis()) > .1 ||

				getRightStickButton() || Math.abs(getRightXAxis()) > .1 || Math.abs(getRightYAxis()) > .1;
	}
	// #endregion
}
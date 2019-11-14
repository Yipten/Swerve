package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class Drivetrain {
	private static final SwerveWheel wheel1;
	private static final SwerveWheel wheel2;
	private static final SwerveWheel wheel3;
	private static final SwerveWheel wheel4;
	private static final ArrayList<SwerveWheel> wheels;

	private static double length;
	private static double width;
	private static double xSpeed;
	private static double ySpeed;
	private static double rotation;

	static {
		// TODO get real numbers before testing (these are fake numbers)
		wheel1 = new SwerveWheel(1, 5, () -> b(), () -> c());
		wheel2 = new SwerveWheel(2, 6, () -> b(), () -> d());
		wheel3 = new SwerveWheel(3, 7, () -> a(), () -> d());
		wheel4 = new SwerveWheel(4, 8, () -> a(), () -> c());
		wheels = new ArrayList<SwerveWheel>();
		wheels.add(wheel1);
		wheels.add(wheel2);
		wheels.add(wheel3);
		wheels.add(wheel4);
	}

	// #region A, B, C, & D
	private static double a() {
		return xSpeed - (rotation * (length / 2));
	}

	private static double b() {
		return xSpeed + (rotation * (length / 2));
	}

	private static double c() {
		return ySpeed - (rotation * (width / 2));
	}

	private static double d() {
		return ySpeed + (rotation * (width / 2));
	}
	// #endregion

	/**
	 * sets the dimensions of the wheelbase
	 * 
	 * @param l length (in.)
	 * @param w width (in.)
	 */
	public static void setDimensions(double l, double w) {
		length = l;
		width = w;
	}

	/**
	 * drives the robot
	 * 
	 * @param x value between -1.0 and 1.0 representing side-to-side movement
	 * @param y value between -1.0 and 1.0 representing forward and backward
	 *          movement
	 * @param r radians/second, positive going clockwise
	 */
	public static void drive(double x, double y, double r) {
		xSpeed = x;
		ySpeed = y;
		rotation = r;
		double largest = 0;
		for (SwerveWheel wheel : wheels) {
			wheel.calc();
			largest = Math.max(wheel.getSpeed(), largest);
		}
		// if any speed values are greater than 1, divide all by the largest
		if (largest > 1)
			for (SwerveWheel wheel : wheels)
				wheel.divSpeed(largest);
		// drive each wheel
		for (SwerveWheel wheel : wheels)
			wheel.drive();
	}

	private static class SwerveWheel {
		// pid stuff for pivoting
		public final PIDController pivotPid;
		public final PIDSource pidSource;
		public final PIDOutput pidOutput;
		// motors
		private final CANSparkMax wheel;
		private final CANSparkMax pivot;
		// calculation functions
		private final Supplier<Double> calcSpeed;
		private final Supplier<Double> calcAngle;

		// values used to drive the SwerveWheel
		private double speed;
		private double angle;
		private double pivotSpeed;

		/**
		 * initializes an instance of the SwerveWheel class
		 * 
		 * @param w     wheel deviceNumber
		 * @param p     pivot deviceNumber
		 * @param func1 function one for calculating speed and angle
		 * @param func2 fuction two for calculating speed and angle
		 */
		public SwerveWheel(int w, int p, Supplier<Double> func1, Supplier<Double> func2) {
			pidSource = new PIDSource() {
				private PIDSourceType pidSourceType;

				@Override
				public void setPIDSourceType(PIDSourceType type) {
					pidSourceType = type;
				}

				@Override
				public PIDSourceType getPIDSourceType() {
					return pidSourceType;
				}

				@Override
				public double pidGet() {
					return pivot.getEncoder().getPosition();	// TODO: get angle in degrees or radians from encoder value
				}
			};

			pidOutput = new PIDOutput() {
				@Override
				public void pidWrite(double output) {
					pivotSpeed = output;
				}
			};

			pidSource.setPIDSourceType(PIDSourceType.kDisplacement);

			pivotPid = new PIDController(0.0001, 0, 0, pidSource, pidOutput);
			pivotPid.setInputRange(-180, 180);
			pivotPid.setOutputRange(-1, 1);
			pivotPid.setContinuous(true);
			pivotPid.setSetpoint(0);

			wheel = new CANSparkMax(w, MotorType.kBrushless);
			pivot = new CANSparkMax(p, MotorType.kBrushless);

			calcSpeed = () -> Math.sqrt(Math.pow(func1.get(), 2) + Math.pow(func2.get(), 2));
			calcAngle = () -> Math.toDegrees(Math.atan2(func1.get(), func2.get()));
		}

		/**
		 * updates speed and angle based on assigned functions
		 */
		public void calc() {
			speed = calcSpeed.get();
			angle = calcAngle.get();
		}

		/**
		 * @return current speed of the wheel
		 */
		public double getSpeed() {
			return speed;
		}

		/**
		 * divides speed by given argument
		 * 
		 * @param largest largest speed of all wheels
		 */
		public void divSpeed(double largest) {
			speed /= largest;
		}

		/**
		 * drives the wheel
		 */
		public void drive() {
			// sets new speed
			wheel.set(speed);
			// sets new angle
			pivotPid.setSetpoint(angle);
			pivot.set(pivotSpeed);
		}
	}
}
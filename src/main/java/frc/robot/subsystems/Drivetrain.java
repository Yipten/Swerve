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
	private static final SwerveModule module1;
	private static final SwerveModule module2;
	private static final SwerveModule module3;
	private static final SwerveModule module4;
	private static final ArrayList<SwerveModule> swerveModules;

	private static double length;
	private static double width;
	private static double xSpeed;
	private static double ySpeed;
	private static double rotation;

	static {
		module1 = new SwerveModule(1, 5, () -> b(), () -> c());	// TODO get real numbers before testing (these are fake numbers)
		module2 = new SwerveModule(2, 6, () -> b(), () -> d());
		module3 = new SwerveModule(3, 7, () -> a(), () -> d());
		module4 = new SwerveModule(4, 8, () -> a(), () -> c());
		swerveModules = new ArrayList<SwerveModule>();
		swerveModules.add(module1);
		swerveModules.add(module2);
		swerveModules.add(module3);
		swerveModules.add(module4);
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
	 * Sets dimensions of the wheelbase.
	 * 
	 * @param l Length in inches.
	 * @param w Width in inches.
	 */
	public static void setDimensions(double l, double w) {
		length = l;
		width = w;
	}

	/**
	 * Drives the robot.
	 * 
	 * @param x Value between -1.0 and 1.0 representing side-to-side movement.
	 * @param y Value between -1.0 and 1.0 representing forward and backward movement.
	 * @param r Radians/second, positive going clockwise.
	 */
	public static void drive(double x, double y, double r) {
		xSpeed = x;
		ySpeed = y;
		rotation = r;
		double largest = 0;
		for (SwerveModule wheel : swerveModules) {
			wheel.calc();
			largest = Math.max(wheel.getSpeed(), largest);
		}
		// if any speed values are greater than 1, divide all by the largest
		if (largest > 1)
			for (SwerveModule wheel : swerveModules)
				wheel.divSpeed(largest);
		// drive each wheel
		for (SwerveModule wheel : swerveModules)
			wheel.drive();
	}

	/**
	 * Nested class for instantiating the four swerve wheels.
	 */
	private static class SwerveModule {
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

		// values used to drive the module
		private double speed;
		private double angle;
		private double pivotSpeed;

		/**
		 * Initializes an instance of the SwerveModule class.
		 * 
		 * @param w     Wheel device number.
		 * @param p     Pivot device number.
		 * @param func1 Function one for calculating speed and angle.
		 * @param func2 Fuction two for calculating speed and angle.
		 */
		public SwerveModule(int w, int p, Supplier<Double> func1, Supplier<Double> func2) {
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
			pidSource.setPIDSourceType(PIDSourceType.kDisplacement);

			pidOutput = new PIDOutput() {
				@Override
				public void pidWrite(double output) {
					pivotSpeed = output;
				}
			};

			pivotPid = new PIDController(0.0001, 0, 0, pidSource, pidOutput);	// TODO: tune these values
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
		 * Updates speed and angle based on assigned functions.
		 */
		public void calc() {
			speed = calcSpeed.get();
			angle = calcAngle.get();
		}

		/**
		 * @return Current speed of the wheel.
		 */
		public double getSpeed() {
			return speed;
		}

		/**
		 * Divides speed by given argument.
		 * 
		 * @param largest Largest speed of all wheels.
		 */
		public void divSpeed(double largest) {
			speed /= largest;
		}

		/**
		 * Drives the module.
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
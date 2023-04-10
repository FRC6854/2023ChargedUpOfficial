package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.helpers.ArmState;
import frc.lib.helpers.NeopixelState;
import frc.lib.helpers.SwerveModuleConstants;

public final class Constants {

	public static final boolean DEBUG = true;

	public static final double looperPeriodSecs = 0.02; // 20ms
	public static final double stickDeadband = 0.18;
	public static final double triggerDeadband = 0.1;

	public static final class FieldConstants {
		public static final double length = Units.feetToMeters(54);
		public static final double width = Units.feetToMeters(27);
	}

	public static final class NeopixelConstants {
		public static final int pwmHeader = 9; // The header on RoboRIO
		public static final int pwmDrive = 46 + 24; // The amount of neopixels to drive
		public static final boolean enabled = true; // To enable it or not, could hurt PWM driver?
		public static final int statusEnding = 14; // Status and General are daisy chained with status starting first
		public static final int firstEnding = 46; // Front amount of pixels
		public static final int backBand = 12; // Back row of pixels (will infer twice)
		public static final int blinkWhen = 15;


		// Normal Debug States
		public static final NeopixelState noneState = new NeopixelState(0, 0, 0);
		public static final NeopixelState startupState = new NeopixelState(0, 255, 0); // Green on startup
		public static final NeopixelState zeroState = new NeopixelState(255, 165, 255); // If its zeroing, show orange
		public static final NeopixelState errorState = new NeopixelState(255, 255, 0); // Wont be used unless something
																						// is wrong

		// Alliance Status States
		public static final NeopixelState redAllianceState = new NeopixelState(255, 0, 0); // Red if getAlliance gives
																							// red
		public static final NeopixelState blueAllianceState = new NeopixelState(0, 0, 255); // Blue if getAlliance gives
																							// blue

		// Current Game Piece States
		public static final NeopixelState cubeState = new NeopixelState(145, 0, 255); // If we are holding a cube, show
																						// purple
		public static final NeopixelState coneState = new NeopixelState(255, 255, 0); // If we are holding a cone, show
																						// yellow

		// Balance Lights
		public static final NeopixelState approachState = new NeopixelState(255, 0, 137); // If we are approaching, show
																							// pink
		public static final NeopixelState driveState = new NeopixelState(0, 255, 255); // If we are driving, show cyan
		public static final NeopixelState balanceState = new NeopixelState(255, 255, 255); // If we are balancing, show
																							// white
	}

	public static final class VisionConstants {
		public static final Transform3d robotToCam = new Transform3d(
				new Translation3d(0.20, 0.135, 0.53),
				new Rotation3d(
						0, 0, 0));
		// from center.
		public static final String camera = "CAM-1";
	}

	public static final class Arm {
		// TODO: Determine Angle Positions, keep tweaking
		public static ArmState collapsedState = new ArmState(1, 1, -2); // When the arm is collapsed
		public static ArmState pickupState = new ArmState(1, 1, 46); // On the ground
		public static ArmState substationAcceptState = new ArmState(1, 1,9);
		public static ArmState level1CubeState = new ArmState(20, 20, 20);
		public static ArmState level2CubeState = new ArmState(37, 74, 28); // 37,68,28
		public static ArmState level1ConeState = new ArmState(37, 65, 31);
		public static ArmState level2ConeState = new ArmState(40, 76, 30);

		public static final double lowerArmGearRatio = (75.0 / 1.0);
		public static final double upperArmGearRatio = (60.0 / 1.0);
		public static final double grabberAngleGearRatio = (125.0 / 1.0);
		public static final double grabberGearRatio = (180.0 / 7.0);

		public static final double lowerArmPauseTime = 0.8;
		public static final double lowerArmConePauseTime = 1.0;

		/*
		 * The lower arm control, mounted on the plate which is mounted on the swerve
		 * modules FL / FR
		 */
		public static final class LowerArm {
			public static final int motorID = 1;
			public static final boolean inverted = false;
			public static final double motorKP = 0.00022;
			public static final double motorKI = 0.0;
			public static final double motorKD = 0.0;
			public static final double motorKF = 0.0;
			public static final int currentLimit = 30;
			public static final int freeCurrentLimit = 30;
			public static final float forwardLimit = 50;
			public static final float reverseLimit = 1;
			public static final double maxVelocity = 4500;
			public static final double maxAcceleration = 4500;
			public static final double slowMaxVelocity = 2000;
			public static final double slowMaxAcceleration = 2500;
		}

		/* Mounted in LowerArm for upper arm */
		public static final class UpperArm {
			public static final int motorID = 2;
			public static final boolean inverted = false;
			public static final double motorKP = 0.00022;
			public static final double motorKI = 0.0;
			public static final double motorKD = 0.0;
			public static final double motorKF = 0.0;
			public static final int currentLimit = 25;
			public static final int freeCurrentLimit = 30;
			public static final float forwardLimit = 100;
			public static final float reverseLimit = 1;
			public static final double maxVelocity = 4500;
			public static final double maxAcceleration = 4500;
		}

		/* For the grabber up / down */
		public static final class GrabberAngle {
			// TODO: Tune Constants
			public static final int motorID = 3;
			public static boolean inverted = false;
			public static final double motorKP = 0.0005;
			public static final double motorKI = 0.0;
			public static final double motorKD = 0.0;
			public static final double motorKF = 0.0;
			public static final int currentLimit = 25;
			public static final int freeCurrentLimit = 30;
			public static final float forwardLimit = 50;
			public static final float reverseLimit = -3;
			public static final double maxVelocity = 4000;
			public static final double maxAcceleration = 4000;
		}

		/* For the intake motor */
		public static final class Intake {
			public static final int motorID = 4;
			public static boolean inverted = true;
			public static final int currentLimit = 20;
			public static final int freeCurrentLimit = 25;
			public static final double intakeSpeed = 0.5;
			public static final double outtakeSpeed = -0.12;
			public static final double holdSpeed = 0.0;
			public static final double ejectSpeed = -0.75;
			public static final double intakeStallCurrent = 10.0;
		}
	}

	public static final class Swerve {
		public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

		public static final double swerveMultiplierSlow = 0.5; // Slow Mode Multiplier

		/** TODO: Tune Auto Constants */
		public static final class AutoConstants {
			public static final double kMaxSpeedMetersPerSecond = 4.5;
			public static final double kMaxAccelerationMetersPerSecondSquared = 1.25;

			public static final double speedFast = 0.4;
			public static final double speedSlow = 0.3; // was 0.2
			public static final double speedBalance = 0.14;
			public static final double onChargeStationDegree = 13.0;
			public static final double levelDegree = 6.0;
			public static final double debounceTime = 0.2;

			public static final PathConstraints kAutoConstraints = new PathConstraints(kMaxSpeedMetersPerSecond,
					kMaxAccelerationMetersPerSecondSquared);

			public static final PIDConstants kTranslationPID = new PIDConstants(6.0, 0.0, 0.0);
			public static final PIDConstants kRotationPID = new PIDConstants(6.0, 0.0, 0.0);
		}

		public static final double wheelDiameter = Units.inchesToMeters(4.0);
		public static final double wheelCircumference = wheelDiameter * Math.PI;
		public static final double angleGearRatio = (12.0 / 1.0);
		public static final double driveGearRatio = (40.0 / 7.0);

		public static final double trackWidth = Units.inchesToMeters(16.0625);
		public static final double wheelBase = Units.inchesToMeters(22);

		public static final double absEncoderGearRatio = (3.0 / 1.0);

		/* Swerve Kinematics */
		public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
				new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
				new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
				new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
				new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

		/* Swerve Current Limiting */
		public static final int angleContinuousCurrentLimit = 25;
		public static final int anglePeakCurrentLimit = 40;
		public static final double anglePeakCurrentDuration = 0.1;
		public static final boolean angleEnableCurrentLimit = true;

		public static final int driveContinuousCurrentLimit = 35;
		public static final int drivePeakCurrentLimit = 55;
		public static final double drivePeakCurrentDuration = 0.1;
		public static final boolean driveEnableCurrentLimit = true;

		/* Drive Motor PID Values */
		public static final double driveKP = 0.08;
		public static final double driveKI = 0.0;
		public static final double driveKD = 0.0;
		public static final double driveKF = 0.0;

		public static final double angleKP = 0.16; // 0.15
		public static final double angleKI = 0.0;
		public static final double angleKD = 0.0001;
		public static final double angleKF = 0.0;
		public static final double angleToleranceDegrees = 0.025;

		public static final double angleVelKP = 0.01;
		public static final double angleVelKI = 0.00001;
		public static final double angleVelKD = 0.01;
		public static final double angleVelKF = 0.0;

		/*
		 * Drive Motor Characterization Values Divide SYSID values by 12 to convert from
		 * volts to percent output for CTRE
		 */
		public static final double driveKS = (0.11758333333 / 12);
		public static final double driveKV = (0.01402966666 / 12);
		public static final double driveKA = (0.00122153333 / 12);

		/* Swerve Profiling Values */
		/** Meters per Second */
		public static final double maxSpeed = 4.5;
		/** Radians per Second */
		public static final double maxAngularVelocity = 10.0;

		/* Swerve Ramp */
		public static final double driveOpenLoopRamp = 0.5;

		/* Neutral Modes */
		public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
		public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

		/* Module Specific Constants */
		/* Front Left Module - Module 0 */
		public static final class Mod0 {
			public static final int driveMotorID = 14;
			public static final int angleMotorID = 15;
			public static final int absEncoderID = 2;
			public static final int opticalSensorID = 7;
			public static final boolean invertDrive = false;
			public static final boolean invertAngle = true;
			public static final double opticalSensorOffset = -45.0; // -92.0
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(70.8);
			public static final SwerveModuleConstants constants = new SwerveModuleConstants(
					driveMotorID, angleMotorID, absEncoderID, opticalSensorID, invertDrive, invertAngle,
					opticalSensorOffset, angleOffset);
		}

		/* Front Right Module - Module 1 */
		public static final class Mod1 {
			public static final int driveMotorID = 12;
			public static final int angleMotorID = 13;
			public static final int absEncoderID = 1;
			public static final int opticalSensorID = 9;
			public static final boolean invertDrive = true;
			public static final boolean invertAngle = true;
			public static final double opticalSensorOffset = 49.33; // -10.0
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(70.125);
			public static final SwerveModuleConstants constants = new SwerveModuleConstants(
					driveMotorID, angleMotorID, absEncoderID, opticalSensorID, invertDrive, invertAngle,
					opticalSensorOffset, angleOffset);
		}

		/* Back Left Module - Module 2 */
		public static final class Mod2 {
			public static final int driveMotorID = 16;
			public static final int angleMotorID = 17;
			public static final int absEncoderID = 3;
			public static final int opticalSensorID = 6;
			public static final boolean invertDrive = false;
			public static final boolean invertAngle = true;
			public static final double opticalSensorOffset = 58.0; // -60.0
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(61.50);
			public static final SwerveModuleConstants constants = new SwerveModuleConstants(
					driveMotorID, angleMotorID, absEncoderID, opticalSensorID, invertDrive, invertAngle,
					opticalSensorOffset, angleOffset);
		}

		/* Back Right Module - Module 3 */
		public static final class Mod3 {
			public static final int driveMotorID = 10;
			public static final int angleMotorID = 11;
			public static final int absEncoderID = 0;
			public static final int opticalSensorID = 8;
			public static final boolean invertDrive = true;
			public static final boolean invertAngle = true;
			public static final double opticalSensorOffset = 299; // 36.45
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(75.17);
			public static final SwerveModuleConstants constants = new SwerveModuleConstants(
					driveMotorID, angleMotorID, absEncoderID, opticalSensorID, invertDrive, invertAngle,
					opticalSensorOffset, angleOffset);
		}
	}
}

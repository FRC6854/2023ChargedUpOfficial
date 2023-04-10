package frc.robot.subsystems.drivetrain;

import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {

	public final SwerveDrivePoseEstimator estimator;
	public final SwerveModule[] swerveModules;

	private final AHRS gyro;

	private final ShuffleboardTab gyroTab = Shuffleboard.getTab("Gyro");
	private final GenericEntry gyroYawEntry;
	private final GenericEntry gyroPitchEntry;
	private final GenericEntry gyroTiltEntry;
	private final GenericEntry gyroPoseYawEntry;

	public final Field2d field = new Field2d();

	private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
	private SwerveModuleState[] lastSetpointStates = new SwerveModuleState[] {
			new SwerveModuleState(),
			new SwerveModuleState(),
			new SwerveModuleState(),
			new SwerveModuleState()
	};

	public SwerveSubsystem() {
		gyro = new AHRS(SPI.Port.kMXP);

		swerveModules = new SwerveModule[] {
				new SwerveModule(0, Constants.Swerve.Mod0.constants, "Front Left"),
				new SwerveModule(1, Constants.Swerve.Mod1.constants, "Front Right"),
				new SwerveModule(2, Constants.Swerve.Mod2.constants, "Back Left"),
				new SwerveModule(3, Constants.Swerve.Mod3.constants, "Back Right")
		};

		/* Calibrate then zero gyro */
		gyro.calibrate();

		Timer.delay(1.0);

		if (!gyro.isCalibrating()) {
			gyro.zeroYaw();
		}

		/* Odometry */
		estimator = new SwerveDrivePoseEstimator(
				Constants.Swerve.swerveKinematics,
				getYaw(),
				getModulePositions(),
				new Pose2d(
						Constants.FieldConstants.length / 2,
						Constants.FieldConstants.width / 2,
						new Rotation2d(0)));

		/* Shuffleboard */
		gyroYawEntry = gyroTab.add("Yaw", getYaw().getDegrees())
				.withWidget(BuiltInWidgets.kGyro)
				.getEntry();

		gyroPitchEntry = gyroTab.add("Pitch", getPitch().getDegrees())
				.withWidget(BuiltInWidgets.kGyro)
				.getEntry();

		gyroTiltEntry = gyroTab.add("Tilt", getTilt())
				.withWidget(BuiltInWidgets.kGyro)
				.getEntry();

		gyroPoseYawEntry = gyroTab.add("Pose Yaw", getPose().getRotation().getDegrees())
				.withWidget(BuiltInWidgets.kGyro)
				.getEntry();

		if (Constants.DEBUG)
			SmartDashboard.putData("Field", field);
	}

	/* Chassis State */

	/**
	 * Get the robot's current desired heading based on the chassis speeds
	 * 
	 * @return The robot's current desired heading
	 */
	public Rotation2d getHeading() {
		return new Rotation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
	}

	/**
	 * Get the robot's current absolute speed based on the chassis speeds
	 * 
	 * @return The robot's current absolute speed
	 */
	public double getSpeed() {
		return Math.sqrt(
				Math.pow(chassisSpeeds.vxMetersPerSecond, 2) +
						Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
	}

	/* Module States & Positions Methods */

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

		Logger.getInstance().recordOutput("Swerve/Desired States", desiredStates);

		for (SwerveModule mod : swerveModules) {
			mod.setDesiredState(desiredStates[mod.id], false);
		}
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : swerveModules) {
			states[mod.id] = mod.getState();
		}
		return states;
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : swerveModules) {
			positions[mod.id] = mod.getPosition();
		}
		return positions;
	}

	public double[] getModuleEncoderPositions() {
		double[] positions = new double[4];
		for (SwerveModule mod : swerveModules) {
			positions[mod.id] = mod.getDrivePosition();
		}
		return positions;
	}

	public double[] getModuleEncoderVelocities() {
		double[] velocities = new double[4];
		for (SwerveModule mod : swerveModules) {
			velocities[mod.id] = mod.getDriveVelocity();
		}
		return velocities;
	}

	/* Odometry (Estimator) methods */
	public Pose2d getPose() {
		return estimator.getEstimatedPosition();
	}

	/**
	 * Updates the odometry with the current module positions and gyro angle
	 */
	public void resetOdometry(Pose2d pose) {
		estimator.resetPosition(getYaw(), getModulePositions(), pose);
	}

	/* Zeroing Methods */

	/**
	 * Resets the modules to their absolute positions
	 */
	public void resetModulesToAbsolute() {
		for (SwerveModule mod : swerveModules) {
			mod.resetToAbsolute();
		}
	}

	/**
	 * Resets the modules to their current position
	 */
	public void resetModulesToCurrent() {
		for (SwerveModule mod : swerveModules) {
			mod.resetToCurrent();
		}
	}

	/* Gyro Methods */

	/**
	 * Resets the Yaw of the Gyro to 0
	 * Also resets the odometry to the current pose
	 */
	public void zeroGyro() {
		gyro.zeroYaw();
		resetOdometry(new Pose2d(getPose().getTranslation(), getYaw()));
	}

	/**
	 * Gets the angle of the robot relative to the field
	 * 
	 * @return The angle of the robot relative to the field
	 */
	public Rotation2d getYaw() {
		return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
				: Rotation2d.fromDegrees(gyro.getYaw());
	}

	/**
	 * Gets the pitch of the robot relative to the field
	 * 
	 * @return The pitch of the robot relative to the field
	 */
	public Rotation2d getPitch() {
		return Rotation2d.fromDegrees(gyro.getPitch());
	}

	/**
	 * Gets the roll of the robot relative to the field
	 * 
	 * @return The roll of the robot relative to the field
	 */
	public Rotation2d getRoll() {
		return Rotation2d.fromDegrees(gyro.getRoll());
	}

	/**
	 * Gets the tilt of the robot relative to the field
	 * 
	 * @return The tilt of the robot relative to the field
	 */
	public double getTilt() {
		double pitch = getPitch().getDegrees();
		double roll = getRoll().getDegrees();

		if ((pitch + roll) >= 0)
			return Math.sqrt(Math.pow(pitch, 2) + Math.pow(roll, 2));

		return -Math.sqrt(Math.pow(pitch, 2) + Math.pow(roll, 2));
	}

	/**
	 * Determines if the gyro is connected
	 * 
	 * @return True if the gyro is connected, false otherwise
	 */
	public boolean isConnected() {
		return gyro.isConnected();
	}

	/* Drive Methods */

	/**
	 * Drives the robot to a point on the field
	 * 
	 * @param point
	 *        The point to drive to
	 * @param alliance
	 *        The alliance of the robot
	 * @return The command to drive to the point
	 */
	public CommandBase driveToPoint(PathPoint point, SwerveAutoBuilder autoBuilder) {
		PathPlannerTrajectory traj = PathPlanner.generatePath(
				Constants.Swerve.AutoConstants.kAutoConstraints,
				PathPoint.fromCurrentHolonomicState(getPose(), chassisSpeeds),
				point);

		return autoBuilder.followPath(traj);
	}

	public void drive(Translation2d translation, double rotation, boolean fieldRelative,
			boolean isOpenLoop) {

		chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
				translation.getX(),
				translation.getY(),
				rotation,
				getPose().getRotation())
				: new ChassisSpeeds(
						translation.getX(),
						translation.getY(),
						rotation);

		/* Cheesy Poofs Drift Fix */

		// Offset for update frequency
		Twist2d setpointTwist = new Pose2d()
				.log(
						new Pose2d(
								chassisSpeeds.vxMetersPerSecond * Constants.looperPeriodSecs,
								chassisSpeeds.vyMetersPerSecond * Constants.looperPeriodSecs,
								new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * Constants.looperPeriodSecs)));

		// Apply the offset
		chassisSpeeds = new ChassisSpeeds(
				setpointTwist.dx / Constants.looperPeriodSecs,
				setpointTwist.dy / Constants.looperPeriodSecs,
				setpointTwist.dtheta / Constants.looperPeriodSecs);

		SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

		/*
		 * The math.pow basically makes x(our trigger value) exponential for the trigger
		 */
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

		// Set to last angles if zero
		if (chassisSpeeds.vxMetersPerSecond == 0.0
				&& chassisSpeeds.vyMetersPerSecond == 0.0
				&& chassisSpeeds.omegaRadiansPerSecond == 0) {
			for (int i = 0; i < 4; i++) {
				swerveModuleStates[i] = new SwerveModuleState(0.0, lastSetpointStates[i].angle);
			}
		}
		lastSetpointStates = swerveModuleStates;

		Logger.getInstance().recordOutput("Swerve/Desired States", swerveModuleStates);

		for (SwerveModule mod : swerveModules) {
			mod.setDesiredState(swerveModuleStates[mod.id], isOpenLoop);
		}
	}

	public void stop() {
		drive(new Translation2d(0, 0), 0, false, false);
	}

	@Override
	public void periodic() {
		estimator.update(getYaw(), getModulePositions());

		/* Gyro */
		gyroYawEntry.setValue(getYaw().getDegrees());
		gyroPitchEntry.setValue(getPitch().getDegrees());
		gyroTiltEntry.setValue(getTilt());
		gyroPoseYawEntry.setValue(getPose().getRotation().getDegrees());

		/* Logging */
		Logger.getInstance().recordOutput("Odometry/Robot", getPose());
		Logger.getInstance().recordOutput("Swerve/States", getModuleStates());
		Logger.getInstance().recordOutput("Swerve/Module Positions", getModuleEncoderPositions());
		Logger.getInstance().recordOutput("Swerve/Module Velocities", getModuleEncoderVelocities());
		Logger.getInstance().recordOutput("Gyro/Yaw", getYaw().getRadians());
		Logger.getInstance().recordOutput("Gyro/Pitch", getPitch().getRadians());
		Logger.getInstance().recordOutput("Gyro/Roll", getRoll().getRadians());
		Logger.getInstance().recordOutput("Gyro/Tilt", getTilt());

		/* Debugging */
		if (Constants.DEBUG) {
			/* Field */
			field.setRobotPose(getPose());

			/* Module States */
			for (SwerveModule mod : swerveModules) {
				mod.updateEntries();
			}
		}
	}
}

package frc.robot.subsystems.drivetrain;

import frc.robot.Constants;
import frc.lib.helpers.SwerveModuleConstants;
import frc.lib.math.Conversions;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveModule {

	public final int id;
	private final String name;

	private final WPI_TalonFX m_driveMotor;
	private final WPI_TalonFX m_angleMotor;

	private final DigitalInput opticalSensor;
	private final DutyCycleEncoder absEnc;

	private final ShuffleboardTab tab;
	private final GenericEntry stateAngleEntry;
	private final GenericEntry velocityEntry;
	private final GenericEntry anglePositionEntry;
	private final GenericEntry opticalEntry;
	private final GenericEntry angleCurrentDrawEntry;
	private final GenericEntry driveCurrentDrawEntry;

	private final Rotation2d angleOffset;
	private final double opticalSensorOffset;
	private final boolean driveMotorInverted;
	private final boolean angleMotorInverted;

	private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
			Constants.Swerve.driveKS,
			Constants.Swerve.driveKV,
			Constants.Swerve.driveKA);

	public SwerveModule(int id, SwerveModuleConstants moduleConstants, String name) {

		this.id = id;
		this.name = name;
		this.angleOffset = moduleConstants.angleOffset;
		this.driveMotorInverted = moduleConstants.invertDrive;
		this.angleMotorInverted = moduleConstants.invertAngle;
		this.opticalSensorOffset = moduleConstants.opticalSensorOffset;

		this.m_driveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
		this.m_angleMotor = new WPI_TalonFX(moduleConstants.angleMotorID);
		this.absEnc = new DutyCycleEncoder(moduleConstants.boreEncoderID);
		this.opticalSensor = new DigitalInput(moduleConstants.opticalSensorID);
		// Reset all of the settings on startup
		m_angleMotor.configFactoryDefault();

		// angle Motor Config
		TalonFXConfiguration angleConfig = new TalonFXConfiguration();
		angleConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		angleConfig.slot0.kP = Constants.Swerve.angleKP;
		angleConfig.slot0.kI = Constants.Swerve.angleKI;
		angleConfig.slot0.kD = Constants.Swerve.angleKD;
		angleConfig.slot0.kF = Constants.Swerve.angleKF;
		angleConfig.slot0.allowableClosedloopError = Conversions.degreesToFalcon(Constants.Swerve.angleToleranceDegrees,
				Constants.Swerve.angleGearRatio);

		angleConfig.slot1.kP = Constants.Swerve.angleVelKP;
		angleConfig.slot1.kI = Constants.Swerve.angleVelKI;
		angleConfig.slot1.kD = Constants.Swerve.angleVelKD;
		angleConfig.slot1.kF = Constants.Swerve.angleVelKF;

		angleConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
				Constants.Swerve.angleEnableCurrentLimit,
				Constants.Swerve.angleContinuousCurrentLimit,
				Constants.Swerve.anglePeakCurrentLimit,
				Constants.Swerve.anglePeakCurrentDuration);
		this.m_angleMotor.configAllSettings(angleConfig);

		m_angleMotor.setInverted(this.angleMotorInverted);
		m_angleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);

		// Reset to the current Talon FX sensor position
		resetToCurrent();

		m_driveMotor.configFactoryDefault();

		// angle Motor Config
		TalonFXConfiguration driveConfig = new TalonFXConfiguration();
		driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		driveConfig.slot0.kP = Constants.Swerve.driveKP;
		driveConfig.slot0.kI = Constants.Swerve.driveKI;
		driveConfig.slot0.kD = Constants.Swerve.driveKD;
		driveConfig.slot0.kF = Constants.Swerve.driveKF;
		driveConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
				Constants.Swerve.driveEnableCurrentLimit,
				Constants.Swerve.driveContinuousCurrentLimit,
				Constants.Swerve.drivePeakCurrentLimit,
				Constants.Swerve.drivePeakCurrentDuration);
		driveConfig.openloopRamp = Constants.Swerve.driveOpenLoopRamp;
		this.m_driveMotor.configAllSettings(driveConfig);
		m_driveMotor.setInverted(this.driveMotorInverted);
		m_driveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);

		/* Shuffleboard */
		this.tab = Shuffleboard.getTab(getName(false));
		this.stateAngleEntry = tab.add("SwerveModuleState Angle", getState().angle.getDegrees())
				.withPosition(0, 0)
				.withSize(2, 2)
				.withWidget(BuiltInWidgets.kGyro).getEntry();
		this.velocityEntry = tab.add("Velocity", getState().speedMetersPerSecond)
				.withWidget(BuiltInWidgets.kDial)
				.withPosition(0, 2)
				.withSize(2, 2)
				.withProperties(Map.of("min", 0, "max", Constants.Swerve.maxSpeed))
				.getEntry();
		this.opticalEntry = tab.add("Optical Sensor", getOpticalSensor()).withWidget(BuiltInWidgets.kBooleanBox)
				.withPosition(2, 0)
				.withSize(2, 2)
				.getEntry();
		this.anglePositionEntry = tab.add("Angle Position", m_angleMotor.getSelectedSensorPosition())
				.withSize(1, 1)
				.withPosition(2, 3)
				.getEntry();
		this.angleCurrentDrawEntry = tab.add("Angle Current Draw", m_angleMotor.getSupplyCurrent())
				.withSize(1, 1)
				.withPosition(2, 2)
				.getEntry();
		this.driveCurrentDrawEntry = tab.add("Drive Current Draw", m_driveMotor.getSupplyCurrent())
				.withSize(1, 1)
				.withPosition(3, 2)
				.getEntry();

		// Zero Drive Motor
		m_driveMotor.setSelectedSensorPosition(0);
	}

	public boolean getOpticalSensor() {
		if (id != 3) {
			return opticalSensor.get();
		}
		return !opticalSensor.get(); // mod 3 is switch
	}

	public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
		desiredState = optimize(desiredState, getState().angle);
		setAngle(desiredState);
		setSpeed(desiredState, isOpenLoop);
	}

	private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
		if (isOpenLoop) {
			double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
			m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
		}
		else {
			double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
					Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);

			m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
					feedforward.calculate(desiredState.speedMetersPerSecond));
		}
	}

	public void setAngleMotorVel(double revPerSec) {
		double vel = Conversions.degreesToFalcon(360 * revPerSec, Constants.Swerve.angleGearRatio);
		m_angleMotor.selectProfileSlot(1, 0);
		m_angleMotor.set(ControlMode.Velocity, vel / 10.0);
	}

	private void setAngle(SwerveModuleState desiredState) {
		m_angleMotor.selectProfileSlot(0, 0);
		m_angleMotor.set(ControlMode.Position,
				Conversions.degreesToFalcon(desiredState.angle.getDegrees(), Constants.Swerve.angleGearRatio));
	}

	private Rotation2d getAngle() {
		return Rotation2d.fromDegrees(
				Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
	}

	public Rotation2d getBoreEncoderRotation() {
		return Rotation2d.fromDegrees(
				Conversions.boreEncoderToDegrees(absEnc.getAbsolutePosition(), Constants.Swerve.absEncoderGearRatio));
	}

	public double getBoreEncoderAbs() {
		return absEnc.getAbsolutePosition();
	}

	public DutyCycleEncoder getBoreEncoder() {
		return absEnc;
	}

	public void resetToAngle(double angle) {
		m_angleMotor.setSelectedSensorPosition(Conversions.degreesToFalcon(angle, Constants.Swerve.angleGearRatio));
	}

	public void resetToCurrent() {
		m_angleMotor.setSelectedSensorPosition(0);
	}

	public void resetToAbsolute() {
		double absPos = Conversions.degreesToFalcon(
				makePositiveDegrees(getBoreEncoderRotation().minus(angleOffset)),
				Constants.Swerve.angleGearRatio);

		m_angleMotor.setSelectedSensorPosition(absPos);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
				Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference,
						Constants.Swerve.driveGearRatio),
				getAngle());
	}

	public SwerveModuleState getZeroSpeedState() {
		return new SwerveModuleState(0, getState().angle);
	}

	public double getOpticalSensorOffset() {
		return opticalSensorOffset;
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
				Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(),
						Constants.Swerve.wheelCircumference,
						Constants.Swerve.driveGearRatio),
				getAngle());
	}

	public double makePositiveDegrees(double anAngle) {
		double degrees = anAngle;
		degrees = degrees % 360;
		if (degrees < 0.0) {
			degrees = degrees + 360;
		}
		return degrees;

	}

	public double getDrivePosition() {
		return m_driveMotor.getSelectedSensorPosition();
	}

	public double getDriveVelocity() {
		return m_driveMotor.getSelectedSensorVelocity();
	}

	public double makePositiveDegrees(Rotation2d anAngle) {
		return makePositiveDegrees(anAngle.getDegrees());
	}

	public void updateEntries() {
		this.stateAngleEntry.setValue(getState().angle.getDegrees());
		this.velocityEntry.setValue(Math.abs(getState().speedMetersPerSecond));
		this.anglePositionEntry.setValue(m_angleMotor.getSelectedSensorPosition());
		this.opticalEntry.setValue(getOpticalSensor());
		this.angleCurrentDrawEntry.setValue(m_angleMotor.getSupplyCurrent());
		this.driveCurrentDrawEntry.setValue(m_driveMotor.getSupplyCurrent());
	}

	/**
	 * Minimize the change in heading the desired swerve module state would require
	 * by potentially
	 * reversing the direction the wheel spins. Customized from WPILib's version to
	 * include placing
	 * in appropriate scope for CTRE onboard control.
	 *
	 * @param desiredState
	 *        The desired state.
	 * @param currentAngle
	 *        The current module angle.
	 */
	public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
		double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
		double targetSpeed = desiredState.speedMetersPerSecond;
		double delta = targetAngle - currentAngle.getDegrees();
		if (Math.abs(delta) > 90) {
			targetSpeed = -targetSpeed;
			targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
		}
		return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
	}

	/**
	 * @param scopeReference
	 *        Current Angle
	 * @param newAngle
	 *        Target Angle
	 * @return Closest angle within scope
	 */
	private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
		double lowerBound;
		double upperBound;
		double lowerOffset = scopeReference % 360;
		if (lowerOffset >= 0) {
			lowerBound = scopeReference - lowerOffset;
			upperBound = scopeReference + (360 - lowerOffset);
		}
		else {
			upperBound = scopeReference - lowerOffset;
			lowerBound = scopeReference - (360 + lowerOffset);
		}
		while (newAngle < lowerBound) {
			newAngle += 360;
		}
		while (newAngle > upperBound) {
			newAngle -= 360;
		}
		if (newAngle - scopeReference > 180) {
			newAngle -= 360;
		}
		else if (newAngle - scopeReference < -180) {
			newAngle += 360;
		}
		return newAngle;
	}

	public String getName(boolean shortform) {
		return shortform ? String.format("Mod %d", id) : String.format("%s - (%d)", name, id);
	}
}

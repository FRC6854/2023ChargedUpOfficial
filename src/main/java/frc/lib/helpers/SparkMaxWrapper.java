package frc.lib.helpers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

public class SparkMaxWrapper extends CANSparkMax {
	private SparkMaxPIDController controller;
	private RelativeEncoder encoder;
	double closedLoopRotationError = 0.5;

	/**
	 * A Wrapper for REV SparkMAX to reduce declaration lines and clean up code
	 * 
	 * @param deviceId
	 * @param type
	 * @param current
	 * @param freeCurrent
	 * @param inverted
	 * @param forwardLimit
	 * @param reverseLimit
	 * @param velocity
	 * @param acceleration
	 */
	public SparkMaxWrapper(int deviceId, MotorType type, int current, int freeCurrent, boolean inverted) {

		super(deviceId, type);
		restoreFactoryDefaults();
		controller = getPIDController();
		if (type == MotorType.kBrushless) {
			encoder = getEncoder();
		}
		else {
			encoder = getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 28);
			encoder.setPosition(40000.0);
			encoder.setInverted(true);
		}

		setSmartCurrentLimit(current, freeCurrent);
		controller.setOutputRange(-1, 1);
		setInverted(inverted);
		if (deviceId != 4) { // Helleman wants this
			setIdleMode(IdleMode.kBrake);
		}
		else {
			setIdleMode(IdleMode.kCoast);
		}
	}

	/**
	 * A Wrapper for REV SparkMAX to reduce declaration lines and clean up code
	 * 
	 * @param deviceId
	 * @param type
	 * @param current
	 * @param freeCurrent
	 * @param inverted
	 * @param forwardLimit
	 * @param reverseLimit
	 * @param velocity
	 * @param acceleration
	 */
	public SparkMaxWrapper(int deviceId, MotorType type, int current, int freeCurrent, boolean inverted,
			float forwardLimit,
			float reverseLimit, double velocity, double acceleration) {

		super(deviceId, type);
		restoreFactoryDefaults();
		controller = getPIDController();
		if (type == MotorType.kBrushless) {
			encoder = getEncoder();
		}
		else {
			encoder = getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 28);
			encoder.setPosition(40000.0);
			encoder.setInverted(true);
		}

		setSmartCurrentLimit(current, freeCurrent);
		controller.setOutputRange(-1, 1);
		setInverted(inverted);
		enableSoftLimit(SoftLimitDirection.kForward, true);
		setSoftLimit(SoftLimitDirection.kForward, forwardLimit);
		enableSoftLimit(SoftLimitDirection.kReverse, true);
		setSoftLimit(SoftLimitDirection.kReverse, reverseLimit);

		setSmartMotion(velocity, acceleration);
		setIdleMode(IdleMode.kBrake);
	}

	/**
	 * Set the PIDF variables
	 * 
	 * @param p
	 *        kP
	 * @param i
	 *        kI
	 * @param d
	 *        kD
	 * @param f
	 *        kF
	 */
	public void setPIDF(double p, double i, double d, double f) {
		controller.setP(p);
		controller.setI(i);
		controller.setD(d);
		controller.setFF(f);
	}

	public void setSmartMotion(double maxVelocity, double maxAcceleration) {
		controller.setSmartMotionMaxVelocity(maxVelocity, 0);
		controller.setSmartMotionMaxAccel(maxAcceleration, 0);
		controller.setSmartMotionAllowedClosedLoopError(closedLoopRotationError, 0);
	}

	public double getPosition() {
		return encoder.getPosition();
	}

	public void smartMotionControl(double setpoint) {
		controller.setReference(setpoint, ControlType.kSmartMotion);
	}

	public void positionControl(double setpoint) {
		controller.setReference(setpoint, ControlType.kPosition);
	}
}

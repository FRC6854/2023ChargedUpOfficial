package frc.lib.helpers;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {

	public final int driveMotorID;
	public final int angleMotorID;
	public final int boreEncoderID;
	public final int opticalSensorID;
	public final boolean invertDrive;
	public final boolean invertAngle;
	public final double opticalSensorOffset;
	public final Rotation2d angleOffset;

	/**
	 * Swerve Module Constants to be used when creating swerve modules.
	 * 
	 * @param driveMotorID
	 * @param angleMotorID
	 * @param boreCoderID
	 * @param angleOffset
	 */
	public SwerveModuleConstants(int driveMotorID, int angleMotorID, int boreEncoderID, int opticalSensorID,
			boolean invertDrive, boolean invertAngle, double opticalSensorOffset, Rotation2d angleOffset) {
		this.driveMotorID = driveMotorID;
		this.angleMotorID = angleMotorID;
		this.boreEncoderID = boreEncoderID;
		this.opticalSensorID = opticalSensorID;
		this.invertDrive = invertDrive;
		this.invertAngle = invertAngle;
		this.opticalSensorOffset = opticalSensorOffset;
		this.angleOffset = angleOffset;
	}
}
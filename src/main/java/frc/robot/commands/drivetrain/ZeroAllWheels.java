package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.light.NeopixelDriver;

public class ZeroAllWheels extends CommandBase {

	private final SwerveSubsystem swerve;
	private final NeopixelDriver light;

	private boolean[] ignoreSensor;
	private boolean[] isZeroed;
	private double[] startPos;

	public ZeroAllWheels(SwerveSubsystem swerve, NeopixelDriver light) {
		this.swerve = swerve;
		this.light = light;

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		ignoreSensor = new boolean[swerve.swerveModules.length];
		isZeroed = new boolean[swerve.swerveModules.length];
		startPos = new double[swerve.swerveModules.length];

		for (int i = 0; i < swerve.swerveModules.length; i++) {
			ignoreSensor[i] = swerve.swerveModules[i].getOpticalSensor();
			startPos[i] = swerve.swerveModules[i].getState().angle.getDegrees();
		}
	}

	@Override
	public void execute() {
		for (SwerveModule mod : swerve.swerveModules) {
			if (!isZeroed[mod.id]) {
				mod.setAngleMotorVel(2.5);

				double currentPos = mod.getState().angle.getDegrees();
				double delta = currentPos - startPos[mod.id];

				if (ignoreSensor[mod.id]) {
					ignoreSensor[mod.id] = !(delta > 70);
				}
				
				isZeroed[mod.id] = !ignoreSensor[mod.id] ? mod.getOpticalSensor() : false;

				if (isZeroed[mod.id]) {
					mod.setAngleMotorVel(0.0);
					mod.resetToAngle(mod.getOpticalSensorOffset());
				}
			}

		}
	}

	@Override
	public boolean isFinished() {
		for (boolean b : isZeroed) {
			if (!b) {
				light.setAlliance(Constants.NeopixelConstants.zeroState);
				return false;
			}
		}
		return true;
	}
}

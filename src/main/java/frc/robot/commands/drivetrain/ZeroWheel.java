package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class ZeroWheel extends CommandBase {

	private final SwerveSubsystem swerve;
	private final int module;

	private boolean ignoreSensor;
	private double startPos;

	public ZeroWheel(SwerveSubsystem swerve, int module) {
		this.swerve = swerve;
		this.module = module;
		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		ignoreSensor = swerve.swerveModules[module].getOpticalSensor();
		startPos = swerve.swerveModules[module].getState().angle.getDegrees();
	}

	@Override
	public void execute() {
		swerve.swerveModules[module].setAngleMotorVel(0.25);

		double currentPos = swerve.swerveModules[module].getState().angle.getDegrees();
		double delta = currentPos - startPos;
		ignoreSensor = !(delta > 70);
	}

	@Override
	public boolean isFinished() {
		if (!ignoreSensor) {
			return swerve.swerveModules[module].getOpticalSensor();
		}
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		swerve.swerveModules[module].setAngleMotorVel(0.0);
		swerve.swerveModules[module].resetToCurrent();
	}
}

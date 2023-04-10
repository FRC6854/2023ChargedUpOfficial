package frc.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class TeleopDrive extends CommandBase {

	private SwerveSubsystem s_swerve;
	private DoubleSupplier translationSup;
	private DoubleSupplier strafeSup;
	private DoubleSupplier rotationSup;
	private BooleanSupplier robotCentricSup;
	private BooleanSupplier slowModeSup;
	private SlewRateLimiter translationSlewLimit;
	private SlewRateLimiter strafeSlewLimit;


	private boolean isSlowMode = false;

	/**
	 * 
	 * @param s_swerve
	 * @param translationSup
	 * @param strafeSup
	 * @param rotationSup
	 * @param triggerSpeed
	 * @param robotCentricSup
	 */
	public TeleopDrive(SwerveSubsystem s_swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
			DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier slowModeSup) {
		this.s_swerve = s_swerve;

		this.translationSup = translationSup;
		this.strafeSup = strafeSup;
		this.rotationSup = rotationSup;
		this.robotCentricSup = robotCentricSup;
		this.slowModeSup = slowModeSup;
		translationSlewLimit = new SlewRateLimiter(1);
		strafeSlewLimit = new SlewRateLimiter(1);

		addRequirements(s_swerve);
	}

	@Override
	public void initialize() {
		// Reset Slow Mode whenever command is initialized
		isSlowMode = false;
	}

	@Override
	public void execute() {
		/* Get Values, Deadband */
		double translationVal = -translationSlewLimit.calculate(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband));
		double strafeVal = -strafeSlewLimit.calculate(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband));
		double rotationVal = -MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

		/* Slow Mode */
		if (slowModeSup.getAsBoolean() && !isSlowMode) {
			isSlowMode = true;
		}
		else if (!slowModeSup.getAsBoolean() && isSlowMode) {
			isSlowMode = false;
		}

		SmartDashboard.putBoolean("Slow Mode", isSlowMode);

		/* Drive */
		s_swerve.drive(
				new Translation2d(translationVal, strafeVal)
						.times(Constants.Swerve.maxSpeed * (isSlowMode ? Constants.Swerve.swerveMultiplierSlow : 1)),
				rotationVal * Constants.Swerve.maxAngularVelocity * (isSlowMode ? Constants.Swerve.swerveMultiplierSlow : 1),
				!robotCentricSup.getAsBoolean(),
				true);
	}

	@Override
	public void end(boolean interrupted) {
		s_swerve.stop();
	}
}

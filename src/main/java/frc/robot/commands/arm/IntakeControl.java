package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeControl extends CommandBase {
	private final IntakeSubsystem s_intake;
	private final DoubleSupplier intakeSup;
	private final DoubleSupplier outtakeSup;

	/**
	 * Creates a new IntakeControl.
	 */
	public IntakeControl(IntakeSubsystem s_intake, DoubleSupplier intakeSup, DoubleSupplier outtakeSup) {
		this.s_intake = s_intake;
		addRequirements(s_intake);

		this.intakeSup = intakeSup;
		this.outtakeSup = outtakeSup;
	}

	@Override
	public void execute() {
		double intakeSpeed = -MathUtil.applyDeadband(intakeSup.getAsDouble(), Constants.stickDeadband) / 2.5;
		double outtakeSpeed = -MathUtil.applyDeadband(outtakeSup.getAsDouble(), Constants.stickDeadband) / 8.5;

		runIntake(intakeSpeed, outtakeSpeed);
	}

	public void runIntake(double intakeSpeed, double outtakeSpeed) {
		s_intake.setSpeed(outtakeSpeed - intakeSpeed);
	}

	@Override
	public void end(boolean interrupted) {
		
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}

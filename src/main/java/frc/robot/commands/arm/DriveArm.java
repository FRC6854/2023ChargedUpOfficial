package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class DriveArm extends CommandBase {

	private final ArmSubsystem s_arm;

	public DriveArm(ArmSubsystem s_arm) {
		this.s_arm = s_arm;
		addRequirements(s_arm);
	}

	@Override
	public void execute() {
		s_arm.moveState();
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}

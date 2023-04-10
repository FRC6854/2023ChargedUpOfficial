// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.helpers.enums.ArmStates;
import frc.lib.helpers.enums.IntakeStates;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class AutoSubstationIntake extends CommandBase {

	private final IntakeSubsystem s_intake;
	private final ArmSubsystem s_arm;

	private Debouncer debounce = new Debouncer(1, Debouncer.DebounceType.kRising);

	public AutoSubstationIntake(IntakeSubsystem s_intake, ArmSubsystem s_arm) {
		this.s_intake = s_intake;
		this.s_arm = s_arm;

		addRequirements(s_intake);
		setName("Auto Substation Intake");
	}

	@Override
	public void initialize() {
		debounce.calculate(false);
		s_arm.setState(ArmStates.SUBSTATION_ACCEPT);
	}

	@Override
	public void execute() {
		s_intake.setState(IntakeStates.INTAKE);
	}

	@Override
	public void end(boolean interrupted) {
		s_arm.setState(ArmStates.COLLAPSED);
		s_intake.setState(IntakeStates.HOLD);
	}

	@Override
	public boolean isFinished() {
		return debounce.calculate(s_intake.getFilteredCurrent() > Constants.Arm.Intake.intakeStallCurrent);
	}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.helpers.enums.ArmStates;
import frc.lib.helpers.enums.GamePiece;
import frc.lib.helpers.enums.IntakeStates;
import frc.robot.commands.arm.DriveArm;
import frc.robot.commands.drivetrain.TeleopDrive;
import frc.robot.commands.drivetrain.ZeroAllWheels;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.light.NeopixelDriver;

public class Level1Cube extends ParallelRaceGroup {
	public Level1Cube(SwerveSubsystem s_swerve, ArmSubsystem s_arm, IntakeSubsystem s_intake, NeopixelDriver s_light,
			boolean zero) {
		setName("Level 1 Auto Cube");
		addCommands(
				new ParallelCommandGroup(
						new InstantCommand(() -> {
							s_arm.setGamePiece(GamePiece.CUBE);
							s_arm.setState(ArmStates.LEVEL_1_CUBE);
							s_intake.setState(IntakeStates.HOLD);
						}),
						zero ? new SequentialCommandGroup(
								new ZeroAllWheels(s_swerve, s_light),
								new TeleopDrive(s_swerve, () -> 0, () -> 0, () -> 0,
										() -> false, () -> false))
								: new TeleopDrive(s_swerve, () -> 0, () -> 0, () -> 0,
										() -> false, () -> false)),
				new SequentialCommandGroup(
						new DriveArm(s_arm).withTimeout(3),
						new InstantCommand(() -> s_intake.setState(IntakeStates.OUTTAKE),
								s_intake),
						new DriveArm(s_arm).withTimeout(0.75),
						new InstantCommand(
								() -> {
									s_arm.setState(ArmStates.COLLAPSED);
									s_intake.setState(IntakeStates.STOPPED);
								}, s_arm, s_intake)));
	}
}

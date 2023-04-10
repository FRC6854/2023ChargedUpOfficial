// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class AutoBalance extends CommandBase {

	private final SwerveSubsystem s_swerve;

	private int debounceCount;
	private final boolean approachBackwards;

	private enum AutoBalanceStates {
		APPROACH, DRIVE, BALANCE, DONE
	}

	private AutoBalanceStates currentState = AutoBalanceStates.APPROACH;

	public AutoBalance(SwerveSubsystem s_swerve) {
		this(s_swerve, true);
	}

	public AutoBalance(SwerveSubsystem s_swerve, boolean approachBackwards) {
		this.s_swerve = s_swerve;
		this.approachBackwards = approachBackwards;

		addRequirements(s_swerve);
		setName("AutoBalance");
	}

	private int secondsToTicks(double time) {
		return (int) (time * 50);
	}

	private void drive(double speed) {
		s_swerve.drive(new Translation2d(approachBackwards ? speed : -speed, 0).times(Constants.Swerve.maxSpeed), 0, true, true);
	}

	@Override
	public void initialize() {
		debounceCount = 0;
	}

	@Override
	public void execute() {
		switch (currentState) {
			case APPROACH:
				if (s_swerve.getTilt() > Constants.Swerve.AutoConstants.levelDegree)
					debounceCount++;

				if (debounceCount > secondsToTicks(Constants.Swerve.AutoConstants.debounceTime)) {
					currentState = AutoBalanceStates.DRIVE;
					debounceCount = 0;
					drive(Constants.Swerve.AutoConstants.speedFast);
					break;
				}

				drive(Constants.Swerve.AutoConstants.speedSlow);
				break;

			case DRIVE:
				if (s_swerve.getTilt() < Constants.Swerve.AutoConstants.levelDegree)
					debounceCount++;

				if (debounceCount > secondsToTicks(Constants.Swerve.AutoConstants.debounceTime)) {
					currentState = AutoBalanceStates.BALANCE;
					debounceCount = 0;
					s_swerve.stop();
					break;
				}

				drive(Constants.Swerve.AutoConstants.speedSlow);
				break;

			case BALANCE:
				if (Math.abs(s_swerve.getTilt()) <= Constants.Swerve.AutoConstants.levelDegree / 2)
					debounceCount++;

				if (debounceCount > secondsToTicks(Constants.Swerve.AutoConstants.debounceTime)) {
					currentState = AutoBalanceStates.DONE;
					debounceCount = 0;
					s_swerve.stop();
				}

				if (s_swerve.getTilt() >= Constants.Swerve.AutoConstants.levelDegree)
					drive(Constants.Swerve.AutoConstants.speedBalance);
				else if (s_swerve.getTilt() <= -Constants.Swerve.AutoConstants.levelDegree)
					drive(-Constants.Swerve.AutoConstants.speedBalance);

				break;

			default:
				s_swerve.stop();
		}
	}

	@Override
	public void end(boolean interrupted) {
		s_swerve.stop();
	}

	@Override
	public boolean isFinished() {
		boolean disconnected = !s_swerve.isConnected();

		if (disconnected)
			System.out.println("[AutoBalance] NavX disconnected!");

		return (currentState == AutoBalanceStates.DONE) || disconnected;
	}
}

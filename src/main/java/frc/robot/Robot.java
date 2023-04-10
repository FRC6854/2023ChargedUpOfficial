package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.helpers.enums.TimeStates;

public class Robot extends LoggedRobot {

	private Command m_autonomousCommand;
	private Command m_disabledCommand;
	private RobotContainer m_robotContainer;

	@Override
	public void robotInit() {
		System.out.println("--------------");
		System.out.println("Robot Init");
		System.out.println("--------------");

		m_robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		System.out.println("--------------");
		System.out.println("Disabled");
		System.out.println("--------------");

		m_robotContainer.s_neopixel.default_state = (Constants.NeopixelConstants.startupState);
		m_robotContainer.s_neopixel.blinking = true;
		m_robotContainer.s_neopixel.blinkColour = (Constants.NeopixelConstants.noneState);
		m_robotContainer.s_neopixel.alliance_state = (Constants.NeopixelConstants.noneState);

		if (m_disabledCommand != null) {
			m_disabledCommand.schedule();
		}
	}

	@Override
	public void autonomousInit() {
		System.out.println("--------------");
		System.out.println("Autonomous");
		System.out.println("--------------");

		m_robotContainer.s_neopixel.setState(TimeStates.AUTO);
		m_robotContainer.s_neopixel.fillAlliance();
		m_robotContainer.s_neopixel.blinking = false;

		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// Cancel the disabled command
		if (m_disabledCommand != null) {
			m_disabledCommand.cancel();
		}

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void teleopInit() {
		System.out.println("--------------");
		System.out.println("Teleop");
		System.out.println("--------------");

		m_robotContainer.s_neopixel.setState(TimeStates.TELEOP);
		m_robotContainer.s_neopixel.default_state = Constants.NeopixelConstants.cubeState;
		m_robotContainer.s_neopixel.blinkColour = Constants.NeopixelConstants.noneState;
		m_robotContainer.s_neopixel.blinking = false;

		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		if (m_disabledCommand != null) {
			m_disabledCommand.cancel();
		}
	}

}

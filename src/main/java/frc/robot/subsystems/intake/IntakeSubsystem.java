package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.helpers.SparkMaxWrapper;
import frc.lib.helpers.enums.IntakeStates;
import frc.lib.helpers.enums.TimeStates;
import frc.robot.Constants;
import frc.robot.subsystems.light.NeopixelDriver;

public class IntakeSubsystem extends SubsystemBase {

	private SparkMaxWrapper m_intakeMotor;

	private LinearFilter currentFilter = LinearFilter.movingAverage(10);
	private double filteredCurrent;

	private IntakeStates state = IntakeStates.STOPPED;

	private final ShuffleboardTab intakeTab;
	private final GenericEntry stateEntry;
	private final GenericEntry currentEntry;
	private final GenericEntry filteredCurrentEntry;
	private final NeopixelDriver s_light;

	public IntakeSubsystem(NeopixelDriver s_light) {
		this.s_light = s_light;
		m_intakeMotor = new SparkMaxWrapper(Constants.Arm.Intake.motorID,
				MotorType.kBrushless,
				Constants.Arm.Intake.currentLimit,
				Constants.Arm.Intake.freeCurrentLimit,
				Constants.Arm.Intake.inverted);

		m_intakeMotor.burnFlash();

		intakeTab = Shuffleboard.getTab("Intake");
		stateEntry = intakeTab.add("State", state.name()).getEntry();
		currentEntry = intakeTab.add("Current", 0).getEntry();
		filteredCurrentEntry = intakeTab.add("Filtered Current", 0).getEntry();
	}

	public void setState(IntakeStates state) {
		if (this.state != state) {
			System.out.println("[Intake] Transitioning Intake from " + this.state.name() + " to " + state.name());
			if (state == IntakeStates.INTAKE && s_light.state == TimeStates.TELEOP){
				s_light.blinking = true;
			} else if (s_light.state == TimeStates.TELEOP) {
				s_light.blinking = false;
			}
			this.state = state;
		}
	}

	public IntakeStates getState() {
		return state;
	}

	public double getCurrent() {
		return m_intakeMotor.getOutputCurrent();
	}

	public double getFilteredCurrent() {
		return filteredCurrent;
	}

	public void setSpeed(double speed) {
		m_intakeMotor.set(speed);
	}

	@Override
	public void periodic() {
		filteredCurrent = currentFilter.calculate(getCurrent());
		switch (state) {
			case INTAKE:
				setSpeed(Constants.Arm.Intake.intakeSpeed);
				break;
			case OUTTAKE:
				setSpeed(Constants.Arm.Intake.outtakeSpeed);
				break;
			case EJECT:
				setSpeed(Constants.Arm.Intake.ejectSpeed);
				break;
			case HOLD:
				setSpeed(Constants.Arm.Intake.holdSpeed);
				break;
			case STOPPED:
				setSpeed(0);
				break;
			default:
				break;
		}

		Logger.getInstance().recordOutput("Intake/State", state.name());
		Logger.getInstance().recordOutput("Intake/Current", getCurrent());
		Logger.getInstance().recordOutput("Intake/Filtered Current", getFilteredCurrent());

		stateEntry.setString(state.name());
		currentEntry.setDouble(getCurrent());
		filteredCurrentEntry.setDouble(filteredCurrent);
	}
}

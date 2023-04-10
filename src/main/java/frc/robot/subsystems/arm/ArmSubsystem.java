package frc.robot.subsystems.arm;

import frc.lib.helpers.SparkMaxWrapper;
import frc.lib.helpers.enums.ArmStates;
import frc.lib.helpers.enums.GamePiece;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.subsystems.light.NeopixelDriver;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
	private final ShuffleboardTab armTab;
	private final NeopixelDriver s_light;
	private final GenericEntry lowerPositionEntry;
	private final GenericEntry lowerAngleEntry;
	private final GenericEntry upperPositionEntry;
	private final GenericEntry upperAngleEntry;
	private final GenericEntry grabberAnglePositionEntry;
	private final GenericEntry grabberAngleAngleEntry;

	private final SparkMaxWrapper m_lowerarm;
	private final SparkMaxWrapper m_upperarm;
	private final SparkMaxWrapper m_grabber_angle;

	private ArmStates state = ArmStates.COLLAPSED;
	private GamePiece piece = GamePiece.CUBE;
	private Timer armTimer = new Timer();

	public ArmSubsystem(NeopixelDriver s_light) {
		this.s_light = s_light;
		m_lowerarm = new SparkMaxWrapper(Constants.Arm.LowerArm.motorID, MotorType.kBrushless,
				Constants.Arm.LowerArm.currentLimit,
				Constants.Arm.LowerArm.freeCurrentLimit, Constants.Arm.LowerArm.inverted,
				Constants.Arm.LowerArm.forwardLimit, Constants.Arm.LowerArm.reverseLimit,
				Constants.Arm.LowerArm.maxVelocity, Constants.Arm.LowerArm.maxAcceleration);

		m_upperarm = new SparkMaxWrapper(Constants.Arm.UpperArm.motorID, MotorType.kBrushless,
				Constants.Arm.UpperArm.currentLimit,
				Constants.Arm.UpperArm.freeCurrentLimit, Constants.Arm.UpperArm.inverted,
				Constants.Arm.UpperArm.forwardLimit, Constants.Arm.UpperArm.reverseLimit,
				Constants.Arm.UpperArm.maxVelocity, Constants.Arm.UpperArm.maxAcceleration);

		m_grabber_angle = new SparkMaxWrapper(Constants.Arm.GrabberAngle.motorID, MotorType.kBrushless,
				Constants.Arm.GrabberAngle.currentLimit,
				Constants.Arm.GrabberAngle.freeCurrentLimit, Constants.Arm.GrabberAngle.inverted,
				Constants.Arm.GrabberAngle.forwardLimit, Constants.Arm.GrabberAngle.reverseLimit,
				Constants.Arm.GrabberAngle.maxVelocity, Constants.Arm.GrabberAngle.maxAcceleration);

		m_lowerarm.setPIDF(Constants.Arm.LowerArm.motorKP, Constants.Arm.LowerArm.motorKI,
				Constants.Arm.LowerArm.motorKD,
				Constants.Arm.LowerArm.motorKF);
		m_upperarm.setPIDF(Constants.Arm.UpperArm.motorKP, Constants.Arm.UpperArm.motorKI,
				Constants.Arm.UpperArm.motorKD,
				Constants.Arm.UpperArm.motorKF);
		m_grabber_angle.setPIDF(Constants.Arm.GrabberAngle.motorKP, Constants.Arm.GrabberAngle.motorKI,
				Constants.Arm.GrabberAngle.motorKD, Constants.Arm.GrabberAngle.motorKF);

		// Burn the flash to the motors
		m_lowerarm.burnFlash();
		m_upperarm.burnFlash();
		m_grabber_angle.burnFlash();

		// Start the lower arm timer
		armTimer.start();

		armTab = Shuffleboard.getTab("Arm");
		lowerPositionEntry = armTab.add("Lower Arm Position", m_lowerarm.getPosition()).getEntry();
		lowerAngleEntry = armTab
				.add("Lower Arm Angle",
						Conversions.neoToDegrees(m_lowerarm.getPosition(), Constants.Arm.lowerArmGearRatio))
				.getEntry();
		upperPositionEntry = armTab.add("Upper Arm Position", m_upperarm.getPosition()).getEntry();
		upperAngleEntry = armTab
				.add("Upper Arm Angle",
						Conversions.neoToDegrees(m_upperarm.getPosition(), Constants.Arm.upperArmGearRatio))
				.getEntry();
		grabberAnglePositionEntry = armTab.add("Grabber Angle Position", m_grabber_angle.getPosition()).getEntry();
		grabberAngleAngleEntry = armTab
				.add("Grabber Angle Angle",
						Conversions.neoToDegrees(m_grabber_angle.getPosition(), Constants.Arm.grabberAngleGearRatio))
				.getEntry();
	}

	/**
	 * Cycle between game pieces for the grabber and arm states
	 */
	public void cycleGamePiece(NeopixelDriver s_light) {
		switch (this.piece) {
			case NONE:
				setGamePiece(GamePiece.CUBE);
				break;
			case CONE:
				setGamePiece(GamePiece.CUBE);
				break;
			case CUBE:
				setGamePiece(GamePiece.CONE);
				break;
			default:
				setGamePiece(GamePiece.CUBE);
		}
	}

	public void setGamePiece(GamePiece piece) {
		if (piece != this.piece) {
			System.out.println("[Arm] Transitioning GamePiece from " + this.piece.name() + " to " + piece.name());
			if (piece == GamePiece.CONE) {
				s_light.default_state = Constants.NeopixelConstants.coneState;
			}
			else {
				s_light.default_state = Constants.NeopixelConstants.cubeState;
			}
			this.piece = piece;
		}
	}

	public void setState(ArmStates state) {
		if (state != this.state) {
			System.out.println("[Arm] Transitioning Arm from " + this.state.name() + " to " + state.name());

			// Hopefully this will help with the arm shaking
			if ((this.state == ArmStates.LEVEL_2_CUBE || this.state == ArmStates.LEVEL_2_CONE
					|| this.state == ArmStates.LEVEL_1_CUBE || this.state == ArmStates.LEVEL_1_CONE)
					&& (state == ArmStates.COLLAPSED || state == ArmStates.PICKUP)) {
				m_lowerarm.setSmartMotion(Constants.Arm.LowerArm.slowMaxVelocity,
						Constants.Arm.LowerArm.slowMaxAcceleration);
				armTimer.reset();
				System.out.println("[Arm] Lower Arm is in slow mode");
				System.out.println("[Arm] Resetting armTimer");
			}
			else {
				if ((this.state == ArmStates.COLLAPSED || this.state == ArmStates.PICKUP)
						&& (state == ArmStates.LEVEL_2_CUBE || state == ArmStates.LEVEL_1_CUBE
								|| state == ArmStates.LEVEL_1_CONE || state == ArmStates.LEVEL_2_CONE)) {
					armTimer.reset();
					System.out.println("[Arm] Resetting armTimer");
				}
				m_lowerarm.setSmartMotion(Constants.Arm.LowerArm.maxVelocity, Constants.Arm.LowerArm.maxAcceleration);
			}

			this.state = state;
		}
	}

	public GamePiece getGamePiece() {
		return this.piece;
	}

	public ArmStates getState() {
		return this.state;
	}

	public void manualDriveArm(double lowerArmPos, double upperArmPos, double grabberAnglePos) {
		m_lowerarm.smartMotionControl(m_lowerarm.getPosition() + lowerArmPos);
		m_upperarm.smartMotionControl(m_upperarm.getPosition() + upperArmPos);
		m_grabber_angle.smartMotionControl(m_grabber_angle.getPosition() + grabberAnglePos);
	}

	public void moveState() {
		switch (state) {
			case PICKUP:
				m_upperarm.smartMotionControl(Constants.Arm.pickupState.upperarmPosition);
				m_grabber_angle.smartMotionControl(Constants.Arm.pickupState.grabberAnglePosition);
				if (armTimer.hasElapsed(Constants.Arm.lowerArmPauseTime)) {
					m_lowerarm.smartMotionControl(Constants.Arm.pickupState.lowerarmPosition);
				}
				else {
					m_lowerarm.smartMotionControl(m_lowerarm.getPosition());
				}
				break;
			case SUBSTATION_ACCEPT:
				m_lowerarm.smartMotionControl(Constants.Arm.substationAcceptState.lowerarmPosition);
				m_upperarm.smartMotionControl(Constants.Arm.substationAcceptState.upperarmPosition);
				m_grabber_angle.smartMotionControl(Constants.Arm.substationAcceptState.grabberAnglePosition);
				break;
			case LEVEL_1_CUBE:
				if (armTimer.hasElapsed(Constants.Arm.lowerArmPauseTime)) {
					m_lowerarm.smartMotionControl(Constants.Arm.level1CubeState.lowerarmPosition);
				}
				else {
					m_lowerarm.smartMotionControl(m_lowerarm.getPosition());
				}
				m_upperarm.smartMotionControl(Constants.Arm.level1CubeState.upperarmPosition);
				m_grabber_angle.smartMotionControl(Constants.Arm.level1CubeState.grabberAnglePosition);
				break;
			case LEVEL_1_CONE:
				if (armTimer.hasElapsed(Constants.Arm.lowerArmPauseTime)) {
					m_lowerarm.smartMotionControl(Constants.Arm.level1ConeState.lowerarmPosition);
				}
				else {
					m_lowerarm.smartMotionControl(m_lowerarm.getPosition());
				}
				m_upperarm.smartMotionControl(Constants.Arm.level1ConeState.upperarmPosition);
				m_grabber_angle.smartMotionControl(Constants.Arm.level1ConeState.grabberAnglePosition);
				break;
			case LEVEL_2_CUBE:
				if (armTimer.hasElapsed(Constants.Arm.lowerArmPauseTime)) {
					m_lowerarm.smartMotionControl(Constants.Arm.level2CubeState.lowerarmPosition);
				}
				else {
					m_lowerarm.smartMotionControl(m_lowerarm.getPosition());
				}
				m_upperarm.smartMotionControl(Constants.Arm.level2CubeState.upperarmPosition);
				m_grabber_angle.smartMotionControl(Constants.Arm.level2CubeState.grabberAnglePosition);
				break;
			case LEVEL_2_CONE:
				if (armTimer.hasElapsed(Constants.Arm.lowerArmConePauseTime)) {
					m_lowerarm.smartMotionControl(Constants.Arm.level2ConeState.lowerarmPosition);
				}
				else {
					m_lowerarm.smartMotionControl(m_lowerarm.getPosition());
				}
				m_upperarm.smartMotionControl(Constants.Arm.level2ConeState.upperarmPosition);
				m_grabber_angle.smartMotionControl(Constants.Arm.level2ConeState.grabberAnglePosition);
				break;
			case MANUAL:
				break;
			case COLLAPSED:
			default:
				m_upperarm.smartMotionControl(Constants.Arm.collapsedState.upperarmPosition);
				m_grabber_angle.smartMotionControl(Constants.Arm.collapsedState.grabberAnglePosition);
				if (armTimer.hasElapsed(Constants.Arm.lowerArmPauseTime)) {
					m_lowerarm.smartMotionControl(Constants.Arm.collapsedState.lowerarmPosition);
				}
				else {
					m_lowerarm.smartMotionControl(m_lowerarm.getPosition());
				}
				break;
		}
	}

	@Override
	public void periodic() {
		SmartDashboard.putString("Game Piece", piece.name());
		SmartDashboard.putString("Arm State", state.name());

		Logger.getInstance().recordOutput("Arm/Game Piece", piece.name());
		Logger.getInstance().recordOutput("Arm/Arm State", state.name());

		lowerPositionEntry.setDouble(m_lowerarm.getPosition());
		lowerAngleEntry.setDouble(Conversions.neoToDegrees(m_lowerarm.getPosition(), Constants.Arm.lowerArmGearRatio));
		upperPositionEntry.setDouble(m_upperarm.getPosition());
		upperAngleEntry.setDouble(Conversions.neoToDegrees(m_upperarm.getPosition(), Constants.Arm.upperArmGearRatio));
		grabberAnglePositionEntry.setDouble(m_grabber_angle.getPosition());
		grabberAngleAngleEntry
				.setDouble(
						Conversions.neoToDegrees(m_grabber_angle.getPosition(), Constants.Arm.grabberAngleGearRatio));
	}
}

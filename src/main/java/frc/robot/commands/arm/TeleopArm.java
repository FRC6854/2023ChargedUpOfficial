package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.helpers.enums.ArmStates;
import frc.lib.helpers.enums.TimeStates;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.light.NeopixelDriver;

public class TeleopArm extends CommandBase {
	private final ArmSubsystem s_arm;
	private final NeopixelDriver s_light;
	private final DoubleSupplier lowerArmSup;
	private final DoubleSupplier upperArmSup;
	private final DoubleSupplier grabberAngleSup;

	/**
	 * Creates a new TeleopArm.
	 * 
	 * @param s_arm
	 *        Arm subsystem
	 * @param grabberOpenSup
	 *        Open grabber manually
	 * @param grabberCloseSup
	 *        Close grabber manually
	 */
	public TeleopArm(ArmSubsystem s_arm, NeopixelDriver s_light,
			DoubleSupplier lowerArmSup, DoubleSupplier upperArmSup, DoubleSupplier grabberAngleSup) {
		this.s_arm = s_arm;
		this.s_light = s_light;
		addRequirements(s_arm);

		this.lowerArmSup = lowerArmSup;
		this.upperArmSup = upperArmSup;
		this.grabberAngleSup = grabberAngleSup;
	}

	@Override
	public void execute() {
		double lowerArmMove = -MathUtil.applyDeadband(lowerArmSup.getAsDouble(), Constants.stickDeadband) * 9;
		double upperArmMove = -MathUtil.applyDeadband(upperArmSup.getAsDouble(), Constants.stickDeadband) * 9;

		if (s_light.state == TimeStates.TELEOP){
			switch (s_arm.getGamePiece()) {
				case CONE:
					break;
				case CUBE:
					break;
				case NONE:
					break;
				default:
					break;
			}
		}

		armMove(lowerArmMove, upperArmMove, grabberAnglePos());
	}

	public double grabberAnglePos() {
		if (grabberAngleSup.getAsDouble() == 90) {
			return 6.0;
		}
		else if (grabberAngleSup.getAsDouble() == 270) {
			return -6.0;
		}
		return 0;
	}

	public void armMove(double lowerArmMove, double upperArmMove, double grabberAnglePos) {
		if (lowerArmMove != 0 || upperArmMove != 0 || grabberAnglePos != 0) {
			s_arm.setState(ArmStates.MANUAL);
			s_arm.manualDriveArm(lowerArmMove, upperArmMove, grabberAnglePos);
		}
		else {
			s_arm.moveState();
		}
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}

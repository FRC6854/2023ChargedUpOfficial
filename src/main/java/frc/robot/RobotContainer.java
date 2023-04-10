package frc.robot;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.helpers.enums.ArmStates;
import frc.lib.helpers.enums.AutoTable;
import frc.lib.helpers.enums.GamePiece;
import frc.lib.helpers.enums.IntakeStates;
import frc.robot.commands.arm.AutoGroundIntake;
import frc.robot.commands.arm.AutoSubstationIntake;
import frc.robot.commands.arm.IntakeControl;
import frc.robot.commands.arm.TeleopArm;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.DriveAutoBuilderWithArm;
import frc.robot.commands.auto.Level2Cube;
import frc.robot.commands.drivetrain.TeleopDrive;
import frc.robot.commands.drivetrain.ZeroAllWheels;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.light.NeopixelDriver;

public class RobotContainer {

	/* Controllers */
	private final XboxController driver = new XboxController(0);
	private final XboxController operator = new XboxController(1);

	/* Driver Buttons */
	private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
	private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kX.value);
	private final JoystickButton groundIntake = new JoystickButton(driver, XboxController.Button.kA.value);
	private final JoystickButton substationIntake = new JoystickButton(driver, XboxController.Button.kB.value);
	private final JoystickButton findWheelZeroAngle = new JoystickButton(driver,
			XboxController.Button.kRightBumper.value);
	private final JoystickButton slowButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

	/* Operator Buttons */
	private final JoystickButton outtake = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
	private final JoystickButton collapsedArm = new JoystickButton(operator, XboxController.Button.kA.value);
	private final JoystickButton pickupArm = new JoystickButton(operator, XboxController.Button.kB.value);
	private final JoystickButton level1ToggleArm = new JoystickButton(operator, XboxController.Button.kX.value);
	private final JoystickButton level2ToggleArm = new JoystickButton(operator, XboxController.Button.kY.value);
	private final JoystickButton toggleGamePiece = new JoystickButton(operator,
			XboxController.Button.kRightStick.value);
	private final Trigger substationArm = new Trigger(() -> operator.getPOV() == 0);

	/* Subsystems */
	private final SwerveSubsystem s_drive;
	private final ArmSubsystem s_arm;
	private final IntakeSubsystem s_intake;
	public final NeopixelDriver s_neopixel;

	/* Autonomous */
	private final LoggedDashboardChooser<AutoTable> autoChooser;
	private final HashMap<String, Command> eventMap = new HashMap<>();
	private final SwerveAutoBuilder autoBuilder;

	public RobotContainer() {
		// Configure logging
		configureLogging();

		s_neopixel = new NeopixelDriver();
		s_drive = new SwerveSubsystem();
		s_arm = new ArmSubsystem(s_neopixel);
		s_intake = new IntakeSubsystem(s_neopixel);

		// Configure auto builder
		autoBuilder = new SwerveAutoBuilder(
				s_drive::getPose,
				s_drive::resetOdometry,
				Constants.Swerve.swerveKinematics,
				Constants.Swerve.AutoConstants.kTranslationPID,
				Constants.Swerve.AutoConstants.kRotationPID,
				s_drive::setModuleStates,
				eventMap,
				true,
				s_drive);

		autoChooser = new LoggedDashboardChooser<>("Auto");

		s_drive.setDefaultCommand(new TeleopDrive(s_drive,
				() -> driver.getLeftY(),
				() -> driver.getLeftX(),
				() -> driver.getRightX(),
				() -> robotCentric.getAsBoolean(),
				() -> slowButton.getAsBoolean()));

		s_arm.setDefaultCommand(new TeleopArm(s_arm, s_neopixel,
				() -> operator.getLeftY(),
				() -> operator.getRightY(),
				() -> operator.getPOV()));

		s_intake.setDefaultCommand(new IntakeControl(s_intake,
				() -> operator.getLeftTriggerAxis(),
				() -> operator.getRightTriggerAxis()));

		// Configure NetworkTables
		configureNetworkTables();

		// Configure events
		configureEventMap();

		// Configure auto chooser
		configureAuto();

		// Configure the button bindings
		configureButtonBindings();

		/* Start NeoPixels if enabled */
		if (Constants.NeopixelConstants.enabled)
			this.s_neopixel.start();

		CameraServer.startAutomaticCapture();

		/* Start PathPlannerServer when not at comp */
		if (Constants.DEBUG)
			PathPlannerServer.startServer(5811);
	}

	private void configureNetworkTables() {
		// Subsystems
		SmartDashboard.putData("Drive", s_drive);
		SmartDashboard.putData("Arm", s_arm);
		SmartDashboard.putData("Intake", s_intake);

		// Commands
		ShuffleboardLayout drivetrainCommands = Shuffleboard.getTab("Commands")
				.getLayout("Drivetrain Commands", BuiltInLayouts.kList)
				.withSize(2, 2);

		drivetrainCommands.add("Zero Wheels", new ZeroAllWheels(s_drive, s_neopixel));
		drivetrainCommands.add("Align Single Substation", s_drive.driveToPoint(
				// Estimated Point
				new PathPoint(new Translation2d(14.18, 7.30), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)),
				autoBuilder).until(() -> {
					// Driver takes control
					return Math.abs(MathUtil.applyDeadband(driver.getLeftX(), Constants.stickDeadband)) > 0
							|| Math.abs(MathUtil.applyDeadband(driver.getLeftY(), Constants.stickDeadband)) > 0
							|| Math.abs(MathUtil.applyDeadband(driver.getRightX(), Constants.stickDeadband)) > 0;
				}));
	}

	private void configureLogging() {
		Logger.getInstance().recordMetadata("FRC6854", "2023");

		if (Robot.isReal()) {
			Logger.getInstance().addDataReceiver(new WPILOGWriter("/home/lvuser"));
			Logger.getInstance().addDataReceiver(new NT4Publisher());
		} else {
			Logger.getInstance().addDataReceiver(new WPILOGWriter(""));
			Logger.getInstance().addDataReceiver(new NT4Publisher());
		}

		Logger.getInstance().start();
	}

	private void configureEventMap() {
		/* Arm State Events */
		eventMap.put(ArmStates.COLLAPSED.toString(), new InstantCommand(() -> s_arm.setState(ArmStates.COLLAPSED)));
		eventMap.put(ArmStates.PICKUP.toString(), new InstantCommand(() -> s_arm.setState(ArmStates.PICKUP)));
		eventMap.put(ArmStates.LEVEL_1_CONE.toString(),
				new InstantCommand(() -> s_arm.setState(ArmStates.LEVEL_1_CONE)));
		eventMap.put(ArmStates.LEVEL_1_CUBE.toString(),
				new InstantCommand(() -> s_arm.setState(ArmStates.LEVEL_1_CUBE)));
		eventMap.put(ArmStates.LEVEL_2_CONE.toString(),
				new InstantCommand(() -> s_arm.setState(ArmStates.LEVEL_2_CONE)));
		eventMap.put(ArmStates.LEVEL_2_CUBE.toString(),
				new InstantCommand(() -> s_arm.setState(ArmStates.LEVEL_2_CUBE)));

		/* Intake State Events */
		eventMap.put(IntakeStates.INTAKE.toString(), new InstantCommand(() -> s_intake.setState(IntakeStates.INTAKE)));
		eventMap.put(IntakeStates.OUTTAKE.toString(),
				new InstantCommand(() -> s_intake.setState(IntakeStates.OUTTAKE)));
		eventMap.put(IntakeStates.STOPPED.toString(),
				new InstantCommand(() -> s_intake.setState(IntakeStates.STOPPED)));
		eventMap.put(IntakeStates.HOLD.toString(), new InstantCommand(() -> s_intake.setState(IntakeStates.HOLD)));
		eventMap.put(IntakeStates.EJECT.toString(), new InstantCommand(() -> s_intake.setState(IntakeStates.EJECT)));
		eventMap.put("AUTO_INTAKE", new AutoGroundIntake(s_intake, s_arm));

		/* Game Piece Events */
		eventMap.put(GamePiece.CONE.toString(), new InstantCommand(() -> s_arm.setGamePiece(GamePiece.CONE)));
		eventMap.put(GamePiece.CUBE.toString(), new InstantCommand(() -> s_arm.setGamePiece(GamePiece.CUBE)));
		eventMap.put(GamePiece.NONE.toString(), new InstantCommand(() -> s_arm.setGamePiece(GamePiece.NONE)));
	}

	private void configureAuto() {
		// Configure SendableChooser
		autoChooser.addDefaultOption("Default (Cube)", AutoTable.DEFAULT_CUBE);
		autoChooser.addOption("Drive Default (Cube)", AutoTable.DRIVE_DEFAULT_CUBE);
		autoChooser.addOption("Top (Cube)", AutoTable.TOP_CUBE);
		autoChooser.addOption("Top Dock (Cube)", AutoTable.TOP_CUBE_DOCK);
		autoChooser.addOption("AutoBuilder Top Double (Cube)", AutoTable.TOP_DOUBLE_CUBE_AUTO_BUILDER);
		autoChooser.addOption("Middle (Cube)", AutoTable.MIDDLE_CUBE);
		autoChooser.addOption("Middle Dock (Cube)", AutoTable.MIDDLE_CUBE_DOCK);
		autoChooser.addOption("Bottom (Cube)", AutoTable.BOTTOM_CUBE);
		autoChooser.addOption("Bottom Dock (Cube)", AutoTable.BOTTOM_CUBE_DOCK);
		autoChooser.addOption("AutoBuilder Bottom Double (Cube)", AutoTable.BOTTOM_DOUBLE_CUBE_AUTO_BUILDER);
		autoChooser.addOption("Testing", AutoTable.TESTING);
		autoChooser.addOption("NONE", AutoTable.NONE);

		SmartDashboard.putNumber("Auto Wait Time", 0);
	}

	private void configureButtonBindings() {
		/* Driver Buttons */
		zeroGyro.onTrue(new InstantCommand(() -> s_drive.zeroGyro()));
		findWheelZeroAngle.debounce(0.6).onTrue(new ZeroAllWheels(s_drive, s_neopixel));
		groundIntake.toggleOnTrue(
				new SequentialCommandGroup(
						new AutoGroundIntake(s_intake, s_arm),
						Commands.run(() -> {
							driver.setRumble(RumbleType.kBothRumble, 0.5);
						}).withTimeout(0.5),
						new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 0)))
								.withName("Ground Intake"));
		substationIntake.toggleOnTrue(
				new SequentialCommandGroup(
						new AutoSubstationIntake(s_intake, s_arm),
						Commands.run(() -> {
							driver.setRumble(RumbleType.kBothRumble, 0.5);
						}).withTimeout(0.5),
						new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 0)))
								.withName("Substation Intake"));

		/* Operator Buttons */
		toggleGamePiece.onTrue(new InstantCommand(() -> s_arm.cycleGamePiece(s_neopixel)));
		collapsedArm.onTrue(new InstantCommand(() -> s_arm.setState(ArmStates.COLLAPSED)));
		pickupArm.onTrue(new InstantCommand(() -> s_arm.setState(ArmStates.PICKUP)));
		substationArm.onTrue(new InstantCommand(() -> s_arm.setState(ArmStates.SUBSTATION_ACCEPT)));
		outtake.whileTrue(
				Commands.runEnd(
						() -> s_intake.setState(IntakeStates.OUTTAKE),
						() -> s_intake.setState(IntakeStates.STOPPED),
						s_intake).withName("Outtake"));
		level1ToggleArm.onTrue(new InstantCommand(() -> {
			switch (s_arm.getGamePiece()) {
				case CUBE:
					s_arm.setState(ArmStates.LEVEL_1_CUBE);
					break;
				case CONE:
					s_arm.setState(ArmStates.LEVEL_1_CONE);
					break;
				default:
					s_arm.setState(ArmStates.COLLAPSED);
					break;
			}
		}));
		level2ToggleArm.onTrue(new InstantCommand(() -> {
			switch (s_arm.getGamePiece()) {
				case CUBE:
					s_arm.setState(ArmStates.LEVEL_2_CUBE);
					break;
				case CONE:
					s_arm.setState(ArmStates.LEVEL_2_CONE);
					break;
				default:
					s_arm.setState(ArmStates.COLLAPSED);
					break;
			}
		}));
	}

	public Command getAutonomousCommand() {
		switch (autoChooser.get()) {
			case NONE:
				return new TeleopDrive(s_drive, () -> 0, () -> 0, () -> 0,
						robotCentric, () -> false);
			case DRIVE_DEFAULT_CUBE:
				return new SequentialCommandGroup(
						new Level2Cube(s_drive, s_arm, s_intake, s_neopixel, true),
						new WaitCommand(SmartDashboard.getNumber("Auto Wait Time", 0)),
						new TeleopDrive(s_drive, () -> -0.6, () -> 0, () -> 0,
								robotCentric, () -> false).withTimeout(2.5)).withName("Drive Default Cube");
			case TOP_CUBE:
				return new SequentialCommandGroup(
						new Level2Cube(s_drive, s_arm, s_intake, s_neopixel, true),
						new WaitCommand(SmartDashboard.getNumber("Auto Wait Time", 5)),
						new DriveAutoBuilderWithArm(s_arm, autoBuilder, "Top")).withName("Top Cube");
			case TOP_CUBE_DOCK:
				return new SequentialCommandGroup(
						new Level2Cube(s_drive, s_arm, s_intake, s_neopixel, true),
						new WaitCommand(SmartDashboard.getNumber("Auto Wait Time", 0)),
						new DriveAutoBuilderWithArm(s_arm, autoBuilder, "Top Dock"),
						new AutoBalance(s_drive, false)).withName("Top Cube Dock");
			case TOP_DOUBLE_CUBE_AUTO_BUILDER:
				return new SequentialCommandGroup(
						new Level2Cube(s_drive, s_arm, s_intake, s_neopixel, true),
						new WaitCommand(SmartDashboard.getNumber("Auto Wait Time", 0)),
						new DriveAutoBuilderWithArm(s_arm, autoBuilder, "Top Cube"),
						new InstantCommand(
								() -> {
									s_arm.setState(ArmStates.COLLAPSED);
									s_intake.setState(IntakeStates.STOPPED);
								}, s_arm, s_intake)).withName("Top Double Cube Auto Builder");
			case MIDDLE_CUBE:
				return new SequentialCommandGroup(
						new Level2Cube(s_drive, s_arm, s_intake, s_neopixel, true),
						new WaitCommand(SmartDashboard.getNumber("Auto Wait Time", 0)),
						new DriveAutoBuilderWithArm(s_arm, autoBuilder, "Middle")).withName("Middle Cube");
			case MIDDLE_CUBE_DOCK:
				return new SequentialCommandGroup(
						new Level2Cube(s_drive, s_arm, s_intake, s_neopixel, true),
						new WaitCommand(SmartDashboard.getNumber("Auto Wait Time", 0)),
						new DriveAutoBuilderWithArm(s_arm, autoBuilder, "Middle Dock"),
						new AutoBalance(s_drive, false)).withName("Middle Cube Dock");
			case BOTTOM_CUBE:
				return new SequentialCommandGroup(
						new Level2Cube(s_drive, s_arm, s_intake, s_neopixel, true),
						new WaitCommand(SmartDashboard.getNumber("Auto Wait Time", 0)),
						new DriveAutoBuilderWithArm(s_arm, autoBuilder, "Bottom")).withName("Bottom Cube");
			case BOTTOM_CUBE_DOCK:
				return new SequentialCommandGroup(
						new Level2Cube(s_drive, s_arm, s_intake, s_neopixel, true),
						new WaitCommand(SmartDashboard.getNumber("Auto Wait Time", 0)),
						new DriveAutoBuilderWithArm(s_arm, autoBuilder, "Bottom Dock"),
						new AutoBalance(s_drive, false)).withName("Bottom Cube Dock");
			case BOTTOM_DOUBLE_CUBE_AUTO_BUILDER:
				return new SequentialCommandGroup(
						new Level2Cube(s_drive, s_arm, s_intake, s_neopixel, true),
						new WaitCommand(SmartDashboard.getNumber("Auto Wait Time", 0)),
						new DriveAutoBuilderWithArm(s_arm, autoBuilder, "Bottom Cube"),
						new InstantCommand(
								() -> {
									s_arm.setState(ArmStates.COLLAPSED);
									s_intake.setState(IntakeStates.STOPPED);
								}, s_arm, s_intake)).withName("Bottom Double Cube Auto Builder");
			case TESTING:
				return new ZeroAllWheels(s_drive, s_neopixel);
			default:
				return new Level2Cube(s_drive, s_arm, s_intake, s_neopixel, true);
		}
	}
}

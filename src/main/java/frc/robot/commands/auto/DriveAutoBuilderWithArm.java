package frc.robot.commands.auto;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.DriveArm;
import frc.robot.subsystems.arm.ArmSubsystem;

public class DriveAutoBuilderWithArm extends ParallelRaceGroup {
	public DriveAutoBuilderWithArm(ArmSubsystem s_arm, SwerveAutoBuilder builder,
			String path) {

		Thread thread = new Thread(new Runnable() {
			@Override
			public void run() {
				System.out.println(
						String.format("[Path] Loading %s path...", path));

				List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(path,
						Constants.Swerve.AutoConstants.kAutoConstraints);

				setName("Drive Auto Builder");
				addCommands(
						builder.fullAuto(pathGroup),
						new DriveArm(s_arm));

				Logger.getInstance().recordOutput("Swerve/Trajectory",
						"Path: " + path + " | " + pathGroup.size() + " Trajectories");

				System.out.println(
						String.format("[Path] Loaded %s path!", path));
			}
		});

		thread.start();
	}
}

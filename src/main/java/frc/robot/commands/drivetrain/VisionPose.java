package frc.robot.commands.drivetrain;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.camera.PhotonCameraSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class VisionPose extends CommandBase {

	private final SwerveSubsystem s_swerve;
	private final PhotonCameraSubsystem s_camera;

	private Alliance currentAlliance;

	public VisionPose(SwerveSubsystem s_swerve, PhotonCameraSubsystem s_camera) {
		this.s_swerve = s_swerve;
		this.s_camera = s_camera;

		addRequirements(s_camera);
	}

	public double getVisionDeviation(Optional<EstimatedRobotPose> result) {
		/*
		 * If we have a target present, return the current error of the vision estimated
		 * pose and our estimated pose
		 */
		if (result.isPresent()) {
			EstimatedRobotPose camPose = result.get();
			return s_swerve.estimator.getEstimatedPosition().minus(camPose.estimatedPose.toPose2d()).getTranslation()
					.getNorm();
		}

		return 0;
	}

	@Override
	public void initialize() {
		currentAlliance = DriverStation.getAlliance();
		s_camera.setAlliance(currentAlliance);
	}

	@Override
	public void execute() {
		Optional<EstimatedRobotPose> result = s_camera
				.getEstimatedGlobalPose(s_swerve.estimator.getEstimatedPosition());

		if (DriverStation.getAlliance() != currentAlliance) {
			currentAlliance = DriverStation.getAlliance();
			s_camera.setAlliance(currentAlliance);
		}

		if (result.isPresent()) {
			EstimatedRobotPose camPose = result.get();

			if ((camPose.estimatedPose.getX() != 0 && camPose.estimatedPose.getY() != 0 &&
			/*
			 * Only update if the camera pose is within 1 meter of the estimated position.
			 * This will hopefully remove values that are not realistic.
			 */
					Math.abs(s_swerve.estimator.getEstimatedPosition().getX() - camPose.estimatedPose.getX()) < 1.0 &&
					Math.abs(s_swerve.estimator.getEstimatedPosition().getY() - camPose.estimatedPose.getY()) < 1.0)
					|| RobotState.isDisabled()) {
				s_swerve.estimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
				s_swerve.field.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
			}
		}
		else {
			// move it way off the screen to make it disappear
			s_swerve.field.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
		}
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}

package frc.robot.subsystems.camera;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonCameraSubsystem extends SubsystemBase {
	private PhotonCamera photonCamera;
	private PhotonPoseEstimator photonPoseEstimator;

	public PhotonCameraSubsystem() {
		// Change the name of your camera here to whatever it is in the PhotonVision UI.
		photonCamera = new PhotonCamera(Constants.VisionConstants.camera);

		try {
			// Attempt to load the AprilTagFieldLayout that will tell us where the tags are
			// on the field.
			AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

			// Create pose estimator
			photonPoseEstimator = new PhotonPoseEstimator(
					fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, Constants.VisionConstants.robotToCam);
			photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		}
		catch (IOException e) {
			// The AprilTagFieldLayout failed to load. We won't be able to estimate poses if
			// we don't know
			// where the tags are.
			DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
			photonPoseEstimator = null;
		}
	}

	/**
	 * Gets latest camera data from the photonvision pipeline
	 * 
	 * @return Result from the camera pipeline
	 */
	public PhotonPipelineResult getLatestResult() {
		return photonCamera.getLatestResult();
	}

	/**
	 * Checks if there is a target detected in the pipeline
	 * 
	 * @param result
	 *        True or false if target exists
	 * @return
	 */
	public boolean checkTargetExistence(PhotonPipelineResult result) {
		return result.hasTargets();
	}

	/**
	 * Fetches all targets detected in the photon pipeline
	 * 
	 * @param result
	 *        A list of targets detected by Photon
	 * @return
	 */
	public List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result) {
		return result.getTargets();
	}

	/**
	 * Returns the best possible target in the Photonvision pipeline
	 * 
	 * @param result
	 * @return
	 */
	public PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
		return result.getBestTarget();
	}

	/**
	 * Gets the 3D pose of an april tag relativeto the camera mount (just in case
	 * pose estimator doesn't work)
	 * 
	 * @param target
	 * @return
	 */
	public Transform3d getTag3D(PhotonTrackedTarget target) {
		return target.getBestCameraToTarget();
	}

	/**
	 * Gets the latest latency of the camera pipeline
	 *
	 * @return The latency in milliseconds
	 */
	public double getLatency() {
		return getLatestResult().getLatencyMillis() / 1000.0;
	}

	/**
	 * Sets the alliance for the field layout
	 * 
	 * @param alliance
	 *        The alliance color
	 */
	public void setAlliance(Alliance alliance) {
		if (photonPoseEstimator == null) {
			// The field layout failed to load, so we cannot estimate poses.
			return;
		}

		AprilTagFieldLayout layout = photonPoseEstimator.getFieldTags();

		switch (alliance) {
			case Blue:
				layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
				break;
			case Red:
				layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
				break;
			case Invalid:
			default:
				layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
				break;
		}

		photonPoseEstimator.setFieldTags(layout);
	}

	/**
	 * @param estimatedRobotPose
	 *        The current best guess at robot pose
	 * @return an EstimatedRobotPose with an estimated pose, the timestamp, and
	 *         targets used to create
	 *         the estimate
	 */
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
		if (photonPoseEstimator == null) {
			// The field layout failed to load, so we cannot estimate poses.
			return Optional.empty();
		}
		photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
		return photonPoseEstimator.update();
	}
}

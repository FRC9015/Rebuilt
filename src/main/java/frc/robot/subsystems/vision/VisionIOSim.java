package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOSim implements VisionIO {

  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final Transform3d robotToCamera;
  private final PhotonCameraSim cameraSim;
  private final PhotonCamera camera;

  /**
   * Creates a new VisionIOSim.
   *
   * @param name The name of the camera (must match PhotonVision config name).
   * @param robotToCamera Transform from robot center to camera.
   * @param poseSupplier Supplier for the robot pose used in simulation.
   */
  public VisionIOSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {

    this.poseSupplier = poseSupplier;
    this.robotToCamera = robotToCamera;

    // Initialize vision system once
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Create PhotonCamera (simulated)
    this.camera = new PhotonCamera(name);

    // Configure simulated camera properties
    SimCameraProperties cameraProperties = new SimCameraProperties();
    cameraProperties.setFPS(30);
    cameraProperties.setAvgLatencyMs(30);
    cameraProperties.setLatencyStdDevMs(5);
    cameraProperties.setCalibration(960, 720, Rotation2d.fromDegrees(90));

    // Create camera simulator
    cameraSim = new PhotonCameraSim(camera, cameraProperties);

    // Add camera to vision system
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update simulation with current robot pose
    visionSim.update(poseSupplier.get());

    inputs.connected = true;
    inputs.hasTargets = false;

    List<PoseObservation> observations = new ArrayList<>();
    List<Integer> targetIds = new ArrayList<>();

    for (var result : camera.getAllUnreadResults()) {
      if (!result.hasTargets()) continue;

      inputs.hasTargets = true;
      result.getTargets().forEach(target -> targetIds.add(target.getFiducialId()));

      Pose3d lensPose;
      int tagCount;
      double ambiguity;

      if (result.getMultiTagResult().isPresent()) {
        var mTag = result.getMultiTagResult().get();
        lensPose =
            new Pose3d(
                mTag.estimatedPose.best.getTranslation(), mTag.estimatedPose.best.getRotation());
        tagCount = mTag.fiducialIDsUsed.size();
        ambiguity = mTag.estimatedPose.ambiguity;
      } else {
        var target = result.getBestTarget();
        Optional<Pose3d> tagPose =
            VisionConstants.aprilTagLayout.getTagPose(target.getFiducialId());
        if (tagPose.isEmpty()) continue;
        lensPose = tagPose.get().transformBy(target.getBestCameraToTarget().inverse());
        tagCount = 1;
        ambiguity = target.getPoseAmbiguity();
      }

      Pose3d robotPose = lensPose.transformBy(robotToCamera.inverse());

      observations.add(
          new PoseObservation(
              result.getTimestampSeconds(),
              robotPose,
              ambiguity,
              tagCount,
              result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm(),
              List.of()));
    }

    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
    inputs.targetIDs = targetIds.stream().mapToInt(Integer::intValue).toArray();
  }
}

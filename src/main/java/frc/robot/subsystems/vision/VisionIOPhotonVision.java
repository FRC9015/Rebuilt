package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    List<PoseObservation> observations = new ArrayList<>();

    for (var result : camera.getAllUnreadResults()) {
      if (!result.hasTargets()) continue;

      Pose3d lensPose;
      int tagCount;
      double ambiguity;
      double dist = 0;

      if (result.getMultiTagResult().isPresent()) {
        var mTag = result.getMultiTagResult().get();
        lensPose = new Pose3d(mTag.estimatedPose.best.getTranslation(), mTag.estimatedPose.best.getRotation());
        tagCount = mTag.fiducialIDsUsed.size();
        ambiguity = mTag.estimatedPose.ambiguity;
      } else {
        var target = result.getBestTarget();
        Optional<Pose3d> tagPose = VisionConstants.aprilTagLayout.getTagPose(target.getFiducialId());
        if (tagPose.isEmpty()) continue;
        lensPose = tagPose.get().transformBy(target.getBestCameraToTarget().inverse());
        tagCount = 1;
        ambiguity = target.getPoseAmbiguity();
      }

      // Convert Lens -> Robot
      Pose3d robotPose = lensPose.transformBy(robotToCamera.inverse());

      observations.add(new PoseObservation(
          result.getTimestampSeconds(),
          robotPose,
          ambiguity,
          tagCount,
          result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm(),
          List.of()
      ));
    }
    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
  }
}
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera("FrontCamera");

  @Override
  public void periodic() {
    var results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      Logger.recordOutput("Vision/ObjectDetection/HasResult", false);
      return;
    }

    Logger.recordOutput("Vision/ObjectDetection/HasResult", true);

    PhotonPipelineResult result = results.get(0);

    if (!result.hasTargets()) {
      Logger.recordOutput("Vision/ObjectDetection/HasTargets", false);
      return;
    }

    Logger.recordOutput("Vision/ObjectDetection/HasTargets", true);

    PhotonTrackedTarget target = result.getBestTarget();

    int classId = target.getFiducialId();
    Transform3d camToTarget = target.getBestCameraToTarget();

    Logger.recordOutput("Vision/ObjectDetection/TargetID", classId);
    Logger.recordOutput("Vision/ObjectDetection/X", camToTarget.getX());
    Logger.recordOutput("Vision/ObjectDetection/Y", camToTarget.getY());
    Logger.recordOutput("Vision/ObjectDetection/Z", camToTarget.getZ());
    Logger.recordOutput(
        "Vision/ObjectDetection/YawDegrees", Math.toDegrees(camToTarget.getRotation().getZ()));
  }
}

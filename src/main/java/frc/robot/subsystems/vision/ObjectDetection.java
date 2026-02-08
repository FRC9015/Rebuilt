package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera("FrontCamera");

  @Override
  public void periodic() {
    // PhotonPipelineResult result = camera.getAllUnreadResults().get(0);

    var results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return;
    }

    PhotonPipelineResult result = results.get(0);

    if (!result.hasTargets()) {
      System.out.println("[VISION] No targets detected");
      return;
    }

    PhotonTrackedTarget target = result.getBestTarget();

    int classId = target.getFiducialId();
    Transform3d camToTarget = target.getBestCameraToTarget();

    System.out.printf(
        "[VISION] Target ID: %d | X: %.2f m | Y: %.2f m | Z: %.2f m | Yaw: %.1f°\n",
        classId,
        camToTarget.getX(),
        camToTarget.getY(),
        camToTarget.getZ(),
        Math.toDegrees(camToTarget.getRotation().getZ()));
  }
}

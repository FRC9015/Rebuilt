package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase {

  private final PhotonCamera camera = new PhotonCamera("Wsp");

  private Translation2d lastTranslation = new Translation2d();

  private double smoothedDistanceCm = 0.0;
  private boolean hasMeasurement = false;

  private double getDistanceFromPitch(double pitch) {
    double x = pitch;
    return -0.93193701 * Math.pow(x, 3)
        + 2.15170038 * Math.pow(x, 2)
        - 7.41876578 * x
        + 57.61577731;
  }

  @Override
  public void periodic() {
    var results = camera.getAllUnreadResults();

    if (results.isEmpty()) {
      Logger.recordOutput("Vision/Status", "No result");
    }

    PhotonPipelineResult result = results.get(results.size() - 1);

    if (!result.hasTargets()) {
      Logger.recordOutput("Vision/Status", "No targets");
    }

    PhotonTrackedTarget target = result.getBestTarget();

    double pitch = target.getPitch();
    double rawDistanceCm = getDistanceFromPitch(pitch);

    if (!hasMeasurement) {
      smoothedDistanceCm = rawDistanceCm; // initialize
      hasMeasurement = true;
    } else {
      double alpha = 0.3; // smoothing factor (0 = heavy smoothing, 1 = no smoothing)
      smoothedDistanceCm = (1 - alpha) * smoothedDistanceCm + alpha * rawDistanceCm;
    }

    double yawRad = Math.toRadians(target.getYaw());

    lastTranslation =
        new Translation2d(
            smoothedDistanceCm * Math.cos(yawRad), smoothedDistanceCm * Math.sin(yawRad));

    Logger.recordOutput("Vision/DistanceCmRaw", rawDistanceCm);
    Logger.recordOutput("Vision/DistanceCmSmoothed", smoothedDistanceCm);
    Logger.recordOutput("Vision/YawDeg", target.getYaw());
    Logger.recordOutput("Vision/PitchDeg", pitch);
    Logger.recordOutput("Vision/Translation", lastTranslation);
  }

  public Translation2d getLastTranslation() {
    return lastTranslation;
  }
}

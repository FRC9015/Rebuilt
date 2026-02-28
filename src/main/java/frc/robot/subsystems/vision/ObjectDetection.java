package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase {

  private final PhotonCamera camera = new PhotonCamera("Wsp");

  private Translation2d lastTranslation = new Translation2d();
  private Translation2d previousTranslation = new Translation2d();

  private double smoothedDistanceCm = 0.0;
  private double lastYawDeg = 0.0;

  private boolean hasMeasurement = false;
  private boolean hasTarget = false;
  private double lastSeenTime = 0.0;

  /*
   * Alr, explanation time.
   * So like basically, you have the pitch and then you have the distance.
   * If you have a pitch axis and distance axis.
   * You can basically do a regression to find a formula that maps pitch to distance nicely.
   * So like that's how all these random-ass numbers came from.
   * I'M BATMAN.
   */

  // TODO This equation accounts for the camera lens distortion for the usb driver camera, and needs to be updated for a new camera.
  private double getDistanceFromPitch(double pitch) {
    double x = pitch;
    return -0.93193701 * Math.pow(x, 3)
        + 2.15170038 * Math.pow(x, 2)
        - 7.41876578 * x
        + 57.61577731;
  }

  @Override
  public void periodic() {

    hasTarget = false;

    var results = camera.getAllUnreadResults();

    if (results.isEmpty()) {
      Logger.recordOutput("Vision/Status", "No result");
      return;
    }

    PhotonPipelineResult result = results.get(results.size() - 1);

    if (!result.hasTargets()) {
      Logger.recordOutput("Vision/Status", "No targets");
      return;
    }

    PhotonTrackedTarget target = result.getBestTarget();

    hasTarget = true;
    lastSeenTime = Timer.getFPGATimestamp();

    double pitch = target.getPitch();
    double rawDistanceCm = getDistanceFromPitch(pitch);

    rawDistanceCm = Math.max(0.0, Math.min(rawDistanceCm, 500.0));

    if (!hasMeasurement) {
      smoothedDistanceCm = rawDistanceCm;
      hasMeasurement = true;
    } else {
      double error = Math.abs(rawDistanceCm - smoothedDistanceCm);
      double alpha = error > 20.0 ? 0.1 : 0.4;
      smoothedDistanceCm =
          (1 - alpha) * smoothedDistanceCm + alpha * rawDistanceCm;
    }

    lastYawDeg = target.getYaw();
    double yawRad = Math.toRadians(lastYawDeg);

    previousTranslation = lastTranslation;

    lastTranslation =
        new Translation2d(
            smoothedDistanceCm * Math.cos(yawRad),
            smoothedDistanceCm * Math.sin(yawRad));

    Translation2d velocity =
        lastTranslation.minus(previousTranslation);

    Translation2d predictedTranslation =
        lastTranslation.plus(velocity.times(0.02));

    Logger.recordOutput("Vision/HasTarget", hasTarget);
    Logger.recordOutput("Vision/LastSeenTime", lastSeenTime);
    Logger.recordOutput("Vision/DistanceCmRaw", rawDistanceCm);
    Logger.recordOutput("Vision/DistanceCmSmoothed", smoothedDistanceCm);
    Logger.recordOutput("Vision/YawDeg", lastYawDeg);
    Logger.recordOutput("Vision/PitchDeg", pitch);
    Logger.recordOutput("Vision/Translation", lastTranslation);
    Logger.recordOutput("Vision/PredictedTranslation", predictedTranslation);
    Logger.recordOutput("Vision/ErrorCm", rawDistanceCm - smoothedDistanceCm);
  }

  public Translation2d getLastTranslation() {
    return lastTranslation;
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  public double getLastSeenTime() {
    return lastSeenTime;
  }

  public double getYawDeg() {
    return lastYawDeg;
  }

  public boolean isAligned(double toleranceDeg) {
    return hasTarget && Math.abs(lastYawDeg) < toleranceDeg;
  }
}

/*
 * interpolation stuff
 * 
import numpy as np

EPSILON = 0.15

pitch = np.array([1.2, -0.2, -1.5, -1.75, -1.7, -2.4])
area = np.array([13.1, 10.5, 6.8, 5.8, 4.6, 3.3])
distance = np.array([50, 60, 70, 80, 90, 100])
#pitch = 2.35, area = 26.5, distance = 40

points = np.column_stack((pitch, area))

def gaussian_rbf(r):
    return np.exp(-(EPSILON * r) ** 2)


def compute_weights(points, values):
    n = len(points)
    A = np.zeros((n, n))

    for i in range(n):
        for j in range(n):
            r = np.linalg.norm(points[i] - points[j])
            A[i, j] = gaussian_rbf(r)

    return np.linalg.solve(A, values)


weights = compute_weights(points, distance)

def interpolate(p, a):
    result = 0
    for i in range(len(points))):
        r = np.linalg.norm(np.array([p, a]) - points[i])
        result += weights[i] * gaussian_rbf(r)
    return result

if __name__ == "__main__":
    test_pitch = 2.35
    test_area = 26.5

    print("RBF:", interpolate(test_pitch, test_area))
 * 
 */
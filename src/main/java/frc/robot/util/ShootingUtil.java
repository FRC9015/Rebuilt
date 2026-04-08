package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootingUtil {

  private static final double DEFAULT_LOOP_DT_SECONDS = 0.02;
  private static final double MIN_LOOP_DT_SECONDS = 0.001;
  private static final double MAX_LOOP_DT_SECONDS = 0.1;

  /** Represents the calculated "Virtual" goal data. */
  public static class ShootingShotData {
    public final Translation2d virtualTargetPos;
    public final double virtualDistance;

    public ShootingShotData(Translation2d pos, double dist) {
      this.virtualTargetPos = pos;
      this.virtualDistance = dist;
    }
  }

  public static ShootingShotData calculateVirtualTarget(
      Pose2d robotPose,
      Translation2d realTarget, // This should be the actual goal position
      ChassisSpeeds robotRelativeSpeeds,
      InterpolatingTreeMap<Double, Double> tofLUT) {

    // 1. Convert to Field-Relative Speeds
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());

    // 2. ZERO-VELOCITY GUARD
    // If we are moving slower than 2cm/s, don't do any math.
    // This proves if the offset is a velocity issue or a coordinate issue.
    if (Math.abs(fieldSpeeds.vxMetersPerSecond) < 0.02
        && Math.abs(fieldSpeeds.vyMetersPerSecond) < 0.02) {
      return new ShootingShotData(realTarget, robotPose.getTranslation().getDistance(realTarget));
    }

    // 3. Get TOF
    double distToRealTarget = robotPose.getTranslation().getDistance(realTarget);
    double tof = tofLUT.get(distToRealTarget);

    // 4. Calculate Offset
    double vXOffset = fieldSpeeds.vxMetersPerSecond * tof;
    double vYOffset = fieldSpeeds.vyMetersPerSecond * tof;

    Translation2d virtualTargetPos =
        new Translation2d(realTarget.getX() - vXOffset, realTarget.getY() - vYOffset);

    return new ShootingShotData(
        virtualTargetPos, robotPose.getTranslation().getDistance(virtualTargetPos));
  }

  /**
   * Estimates scheduler/compute delay from loop timing.
   *
   * <p>Model: half-loop average sample delay + full loops of pipeline staging.
   */
  public static double estimateLoopSchedulerLatencySeconds(
      double measuredLoopDtSeconds, int additionalPipelineLoops) {
    double loopDt = clampLoopDt(measuredLoopDtSeconds);
    int loops = Math.max(0, additionalPipelineLoops);
    return loopDt * (0.5 + loops);
  }

  /**
   * Estimates total aiming latency for moving shots.
   *
   * <p>Use this in your target prediction horizon: predictionTime = tof + totalLatency.
   */
  public static double estimateTotalLatencySeconds(
      double measuredLoopDtSeconds,
      int additionalPipelineLoops,
      double visionMeasurementAgeSeconds,
      double actuatorResponseSeconds,
      double noteReleaseSeconds,
      double biasSeconds) {
    double schedulerLatency =
        estimateLoopSchedulerLatencySeconds(measuredLoopDtSeconds, additionalPipelineLoops);

    return schedulerLatency
        + sanitizeNonNegative(visionMeasurementAgeSeconds)
        + sanitizeNonNegative(actuatorResponseSeconds)
        + sanitizeNonNegative(noteReleaseSeconds)
        + sanitizeNonNegative(biasSeconds);
  }

  /**
   * Exponential smoothing helper for loop dt measurements.
   *
   * <p>alpha in [0, 1], where 1.0 = no smoothing.
   */
  public static double smoothLoopDtSeconds(
      double previousSmoothedLoopDtSeconds, double latestMeasuredLoopDtSeconds, double alpha) {
    double clampedAlpha = Math.min(1.0, Math.max(0.0, alpha));
    double prev = clampLoopDt(previousSmoothedLoopDtSeconds);
    double latest = clampLoopDt(latestMeasuredLoopDtSeconds);
    return (clampedAlpha * latest) + ((1.0 - clampedAlpha) * prev);
  }

  private static double sanitizeNonNegative(double value) {
    if (!Double.isFinite(value) || value < 0.0) {
      return 0.0;
    }
    return value;
  }

  private static double clampLoopDt(double measuredLoopDtSeconds) {
    if (!Double.isFinite(measuredLoopDtSeconds)) {
      return DEFAULT_LOOP_DT_SECONDS;
    }
    return Math.min(MAX_LOOP_DT_SECONDS, Math.max(MIN_LOOP_DT_SECONDS, measuredLoopDtSeconds));
  }
}

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootingUtil {

  /** Represents the calculated "Virtual" goal data. */
  public static class ShootingShotData {
    public final Translation2d virtualTargetPos;
    public final double virtualDistance;

    public ShootingShotData(Translation2d pos, double dist) {
      this.virtualTargetPos = pos;
      this.virtualDistance = dist;
    }
  }

  /**
   * Calculates the virtual target. accounts for TOF and loop-based latency.
   *
   * @param loopDt Seconds per loop (e.g., 0.02)
   * @param latencyCycles Number of loops to compensate for (typically 1.5 to 2.0)
   */
  public static ShootingShotData calculateVirtualTargetLocal(
      Pose2d robotPose,
      Translation2d realTarget,
      ChassisSpeeds robotRelativeSpeeds,
      InterpolatingTreeMap<Double, Double> tofLUT,
      double loopDt,
      double latencyCycles) {

    // 1. Convert to Field-Relative Speeds
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());

    // 2. ZERO-VELOCITY GUARD
    if (Math.abs(fieldSpeeds.vxMetersPerSecond) < 0.02
        && Math.abs(fieldSpeeds.vyMetersPerSecond) < 0.02) {
      return new ShootingShotData(realTarget, robotPose.getTranslation().getDistance(realTarget));
    }

    // 3. Calculate Total Prediction Time (TOF + Loop Latency)
    double distToRealTarget = robotPose.getTranslation().getDistance(realTarget);
    double tof = tofLUT.get(distToRealTarget);
    double totalLatency = loopDt * latencyCycles;
    double lookaheadTime = tof + totalLatency;

    // 4. Calculate Offset
    double vXOffset = fieldSpeeds.vxMetersPerSecond * lookaheadTime;
    double vYOffset = fieldSpeeds.vyMetersPerSecond * lookaheadTime;

    Translation2d virtualTargetPos =
        new Translation2d(realTarget.getX() - vXOffset, realTarget.getY() - vYOffset);

    return new ShootingShotData(
        virtualTargetPos, robotPose.getTranslation().getDistance(virtualTargetPos));
  }

  /** Overload using default FRC constants (20ms loop, 1.5 cycle latency) */
  public static ShootingShotData calculateVirtualTarget(
      Pose2d robotPose,
      Translation2d realTarget,
      ChassisSpeeds robotRelativeSpeeds,
      InterpolatingTreeMap<Double, Double> tofLUT) {
    return calculateVirtualTargetLocal(
        robotPose, realTarget, robotRelativeSpeeds, tofLUT, 0.02, 1.5);
  }
}

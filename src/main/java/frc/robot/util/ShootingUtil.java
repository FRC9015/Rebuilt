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
}

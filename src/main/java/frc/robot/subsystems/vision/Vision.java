package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Shared PhotonVision/Umbra observation validation, visualization, and pose fusion.
 *
 * <p>Vision fusion is OFF by default for controlled Umbra testing. Raw and accepted camera poses
 * are still visible through AdvantageKit/AdvantageScope.
 */
public class Vision extends SubsystemBase {
  // Keep disabled until Umbra's real-world timestamp, mounting, and odometry
  // compatibility have been confirmed. Call setFusionEnabled(true) deliberately.
  private boolean fusionEnabled = true;

  private static final double MAX_REAL_OBSERVATION_AGE_SEC = 0.50;
  private static final double MAX_FUTURE_SEC = 0.05;
  // When Umbra doesn't publish a real tag range, DO NOT use its legacy range value.
  // Deliberately conservative starting uncertainty; tune on actual field data.
  private static final Matrix<N3, N1> UMBRA_UNKNOWN_RANGE_STD_DEVS = VecBuilder.fill(1.0, 1.0, 1.5);

  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final AprilTagFieldLayout aprilTagLayout;

  public Vision(VisionConsumer consumer, AprilTagFieldLayout layout, VisionIO... io) {
    this.consumer = consumer;
    this.aprilTagLayout = layout;
    this.io = io;
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    this.disconnectedAlerts = new Alert[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnectedAlerts[i] =
          new Alert(
              "Vision " + io[i].getName() + " has no recent data/connection.", AlertType.kWarning);
    }
  }

  /** Set true only after validating the received measurements and clock synchronization. */
  public void setFusionEnabled(boolean enabled) {
    fusionEnabled = enabled;
  }

  public boolean isFusionEnabled() {
    return fusionEnabled;
  }

  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  public Optional<PoseObservation> getNewestPoseObservation(int cameraIndex) {
    PoseObservation[] observations = inputs[cameraIndex].poseObservations;
    return observations.length == 0
        ? Optional.empty()
        : Optional.of(observations[observations.length - 1]);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    Logger.recordOutput("Vision/FusionEnabled", fusionEnabled);
    double nowSec = Timer.getFPGATimestamp();
    List<Pose3d> allRawPoses = new ArrayList<>();
    List<Pose3d> allAcceptedPoses = new ArrayList<>();
    List<Pose3d> allRejectedPoses = new ArrayList<>();
    List<Pose3d> allTagPoses = new ArrayList<>();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      String key = "Vision/Camera" + cameraIndex + "/";
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      List<Pose3d> tagPoses = new ArrayList<>();
      List<Pose3d> rawPoses = new ArrayList<>();
      List<Pose3d> acceptedPoses = new ArrayList<>();
      List<Pose3d> rejectedPoses = new ArrayList<>();
      int fusedCount = 0;
      int acceptedCount = 0;
      int rejectedCount = 0;

      for (int tagId : inputs[cameraIndex].tagIds) {
        aprilTagLayout.getTagPose(tagId).ifPresent(tagPoses::add);
      }

      for (PoseObservation observation : inputs[cameraIndex].poseObservations) {
        if (observation == null || observation.pose() == null) {
          rejectedCount++;
          continue;
        }
        Pose3d pose = observation.pose();
        if (isFinitePose(pose)) {
          rawPoses.add(pose);
        }

        boolean reject = !isValidObservation(observation, nowSec);
        if (reject) {
          rejectedCount++;
          if (isFinitePose(pose)) {
            rejectedPoses.add(pose);
          }
          continue;
        }

        double range = observation.averageTagDistance();
        Matrix<N3, N1> stdDevs;
        if (Double.isFinite(range) && range >= 0.0) {
          stdDevs =
              observation.tagCount() > 1
                  ? VisionConstants.kMultiTagStdDevs
                  : VisionConstants.kSingleTagStdDevs;
          stdDevs = stdDevs.times(1.0 + range * range / VisionConstants.STD_DEV_RANGE);
        } else {
          // Umbra does not currently publish camera-to-tag distance.
          // Never weight using the distance to the field origin instead.
          stdDevs = UMBRA_UNKNOWN_RANGE_STD_DEVS;
        }

        acceptedCount++;
        acceptedPoses.add(pose);
        Logger.recordOutput(key + "StdDevs", stdDevs);

        // Raw and accepted measurements are logged even when fusion is disabled.
        if (fusionEnabled) {
          consumer.accept(pose.toPose2d(), observation.timestamp(), stdDevs);
          fusedCount++;
        }
      }

      // ALWAYS write arrays, including empty ones. Otherwise AdvantageScope
      // can keep rendering a previously accepted pose after tags disappear.
      Logger.recordOutput(key + "Name", io[cameraIndex].getName());
      Logger.recordOutput(key + "Connected", inputs[cameraIndex].connected);
      Logger.recordOutput(key + "TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(key + "RobotPoses", rawPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(key + "AcceptedRobotPoses", acceptedPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(key + "RejectedRobotPoses", rejectedPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(key + "ReceivedCount", inputs[cameraIndex].poseObservations.length);
      Logger.recordOutput(key + "AcceptedCount", acceptedCount);
      Logger.recordOutput(key + "RejectedCount", rejectedCount);
      Logger.recordOutput(key + "FusedCount", fusedCount);

      allTagPoses.addAll(tagPoses);
      allRawPoses.addAll(rawPoses);
      allAcceptedPoses.addAll(acceptedPoses);
      allRejectedPoses.addAll(rejectedPoses);
    }

    Logger.recordOutput("Vision/AllTagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/AllRobotPoses", allRawPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/AllAcceptedRobotPoses", allAcceptedPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/AllRejectedRobotPoses", allRejectedPoses.toArray(new Pose3d[0]));
  }

  private boolean isValidObservation(PoseObservation observation, double nowSec) {
    Pose3d pose = observation.pose();
    if (!isFinitePose(pose) || !Double.isFinite(observation.timestamp())) {
      return false;
    }

    if (observation.tagCount() < 1) {
      return false;
    }
    if (observation.tagCount() == 1
        && (!Double.isFinite(observation.ambiguity())
            || observation.ambiguity() > VisionConstants.MAX_AMBIGUITY)) {
      return false;
    }
    if (pose.getX() < 0.0
        || pose.getX() > aprilTagLayout.getFieldLength()
        || pose.getY() < 0.0
        || pose.getY() > aprilTagLayout.getFieldWidth()) {
      return false;
    }
    double dist = observation.averageTagDistance();
    if (observation.tagCount() == 1
        && Double.isFinite(dist)
        && dist > VisionConstants.MAX_AVERAGE_DISTANCE) {
      return false;
    }

    // RobotEstimator timestamps use the roboRIO FPGA clock, in seconds.
    // During simulated/replayed robot runs, clock progression may be different.
    if (Constants.currentMode == Constants.Mode.REAL) {
      double ageSec = nowSec - observation.timestamp();
      if (ageSec < -MAX_FUTURE_SEC || ageSec > MAX_REAL_OBSERVATION_AGE_SEC) {
        return false;
      }
    }
    return true;
  }

  private static boolean isFinitePose(Pose3d pose) {
    return Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getZ())
        && Double.isFinite(pose.getRotation().getX())
        && Double.isFinite(pose.getRotation().getY())
        && Double.isFinite(pose.getRotation().getZ());
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}

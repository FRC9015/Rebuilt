package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Receives field-relative ROBOT poses from Umbra on /Umbra/{camera}/observations.
 *
 * <p>Umbra's current nine-element payload is: [0] capture/retrieval timestamp in NT server seconds,
 * [1..3] field-relative robot X/Y/Z (meters), [4..6] robot roll/pitch/yaw (radians), [7] number of
 * accepted tags, [8] a legacy "distance" (currently NOT tag distance).
 *
 * <p>Do not apply the robot-to-camera mount a second time: Umbra already does so. No data means no
 * new observation. A retained NT value is NOT a new frame.
 */
public class VisionIOUmbra implements VisionIO {
  // Conservative starting limits; retune after measuring real NT and camera latency.
  private static final double MAX_OBSERVATION_AGE_SEC = 0.50;
  private static final double MAX_TRANSPORT_AGE_SEC = 0.50;
  private static final double MAX_FUTURE_SEC = 0.05;
  private static final double OBSERVATION_TIMEOUT_SEC = 1.00;

  private final String cameraName;
  private final DoubleArraySubscriber observationSub;
  private long previousServerTimeUs = 0;
  private double lastPacketTimeSec = Double.NEGATIVE_INFINITY;

  public VisionIOUmbra(String cameraName) {
    this.cameraName = cameraName;
    var topic =
        NetworkTableInstance.getDefault()
            .getTable("Umbra")
            .getSubTable(cameraName)
            .getDoubleArrayTopic("observations");

    // Request a 20 ms network period and queue actual new samples, not the retained value.
    // The publisher may still send at a different negotiated rate.
    observationSub =
        topic.subscribe(
            new double[] {},
            PubSubOption.periodic(0.02),
            PubSubOption.sendAll(true),
            PubSubOption.pollStorage(32));
  }

  @Override
  public String getName() {
    return cameraName;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double nowSec = Timer.getFPGATimestamp();
    List<PoseObservation> observations = new ArrayList<>();
    int invalidSamples = 0;
    int newPackets = 0;
    double latestSourceAgeSec = -1.0;
    double[] latestRawSample = new double[0];

    for (TimestampedDoubleArray sample : observationSub.readQueue()) {
      // On the roboRIO, NT server time and Timer.getFPGATimestamp() share a clock.
      // Both NT timestamps are microseconds; Umbra's payload time is seconds.
      if (sample.serverTime <= 0 || sample.serverTime <= previousServerTimeUs) {
        continue;
      }
      previousServerTimeUs = sample.serverTime;

      double publicationTimeSec = sample.serverTime / 1_000_000.0;
      double transportAgeSec = nowSec - publicationTimeSec;
      if (!Double.isFinite(transportAgeSec)
          || transportAgeSec < -MAX_FUTURE_SEC
          || transportAgeSec > MAX_TRANSPORT_AGE_SEC) {
        invalidSamples++;
        continue;
      }

      lastPacketTimeSec = publicationTimeSec;
      newPackets++;
      double[] data = sample.value;
      latestRawSample = data == null ? new double[0] : data;

      if (data == null || data.length != 9 || !allFinite(data, 0, 8)) {
        invalidSamples++;
        continue;
      }

      // Pi payload timestamp may be in an UNSYNCHRONIZED clock domain at startup.
      // Do not guess, replace it with arrival time, or feed that pose to the estimator.
      double measurementTimeSec = data[0];
      latestSourceAgeSec = nowSec - measurementTimeSec;
      if (measurementTimeSec <= 0.0
          || measurementTimeSec > publicationTimeSec + MAX_FUTURE_SEC
          || publicationTimeSec - measurementTimeSec > MAX_OBSERVATION_AGE_SEC
          || latestSourceAgeSec < -MAX_FUTURE_SEC
          || latestSourceAgeSec > MAX_OBSERVATION_AGE_SEC) {
        invalidSamples++;
        continue;
      }

      double rawTagCount = data[7];
      if (rawTagCount < 2.0 || rawTagCount > 64.0 || Math.rint(rawTagCount) != rawTagCount) {
        // Current Umbra intentionally publishes only multi-tag robot poses.
        invalidSamples++;
        continue;
      }

      Pose3d robotPose =
          new Pose3d(data[1], data[2], data[3], new Rotation3d(data[4], data[5], data[6]));

      observations.add(
          new PoseObservation(
              measurementTimeSec,
              robotPose,
              0.0, // No ambiguity metric in Umbra's wire format; NOT a measured zero.
              (int) rawTagCount,
              Double.NaN, // No valid camera-to-tag distance provided by Umbra yet.
              List.of())); // Tag IDs are not included in this version of Umbra's payload.
    }

    // Reset per-loop state, so there is no way to accidentally replay an old observation.
    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
    inputs.tagIds = new int[0];
    inputs.targetIDs = new int[0];
    inputs.hasTargets = !observations.isEmpty();

    // Without a separate heartbeat, this means "recent pose messages", not
    // "the camera is powered on". It may be false when no tags are in view.
    inputs.connected = nowSec - lastPacketTimeSec <= OBSERVATION_TIMEOUT_SEC;

    String logPrefix = "Vision/Umbra/" + cameraName + "/";
    Logger.recordOutput(logPrefix + "NewPackets", newPackets);
    Logger.recordOutput(logPrefix + "ValidObservations", observations.size());
    Logger.recordOutput(logPrefix + "InvalidPackets", invalidSamples);
    Logger.recordOutput(
        logPrefix + "SecondsSinceLastPacket",
        Double.isFinite(lastPacketTimeSec) ? nowSec - lastPacketTimeSec : -1.0);
    Logger.recordOutput(logPrefix + "LastSourceAgeSec", latestSourceAgeSec);
    Logger.recordOutput(logPrefix + "RawObservation", latestRawSample);
  }

  private static boolean allFinite(double[] values, int begin, int endInclusive) {
    for (int i = begin; i <= endInclusive; i++) {
      if (!Double.isFinite(values[i])) {
        return false;
      }
    }
    return true;
  }
}

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Main vision processing subsystem.
 *
 * <p>This class updates all VisionIO implementations, logs their inputs, and processes detected
 * AprilTag observations. It supports both static cameras and dynamic turret-mounted cameras.
 */
public class Vision extends SubsystemBase {

  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  // Turret specifics
  private final Supplier<Rotation2d> turretAngleSupplier;
  private final int turretCameraIndex;

  private Matrix<N3, N1> curStdDevs = VisionConstants.kSingleTagStdDevs;

  /**
   * Creates a new Vision Subsystem WITH Turret Support.
   *
   * @param consumer The consumer (usually your SwerveDrivePoseEstimator)
   * @param turretAngleSupplier A method to get the live angle of the turret (e.g., () ->
   *     turret.getAngle())
   * @param turretCameraIndex The index of the turret camera in the io array (use -1 if no turret
   *     camera exists)
   * @param io The VisionIO instances (e.g., Starboard, Port, Turret)
   */
  public Vision(
      VisionConsumer consumer,
      Supplier<Rotation2d> turretAngleSupplier,
      int turretCameraIndex,
      VisionIO... io) {

    this.consumer = consumer;
    this.turretAngleSupplier = turretAngleSupplier;
    this.turretCameraIndex = turretCameraIndex;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);
    }
  }

  /** Standard constructor without a turret (for testing or basic setups). */
  public Vision(VisionConsumer consumer, VisionIO... io) {
    this(consumer, () -> new Rotation2d(), -1, io);
  }

  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  public Optional<PoseObservation> getNewestPoseObservation(int cameraIndex) {
    PoseObservation[] observations = inputs[cameraIndex].poseObservations;
    if (observations.length != 0) {
      return Optional.of(observations[0]);
    } else {
      return Optional.empty();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses to logs
      for (int tagId : inputs[cameraIndex].tagIds) {
        Optional<Pose3d> tagPose = VisionConstants.aprilTagLayout.getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      // Loop over pose observations
      for (PoseObservation observation : inputs[cameraIndex].poseObservations) {

        // 1. Get the raw pose from the Pi
        Pose3d robotPose = observation.pose();

        // 2. --- DYNAMIC TURRET MATH ---
        // If this is the turret camera, the Pi sent us the CAMERA LENS pose.
        // We must reverse-engineer the robot center based on the live turret angle.
        if (cameraIndex == turretCameraIndex && turretAngleSupplier != null) {
          Rotation2d liveTurretAngle = turretAngleSupplier.get();

          // Transform 1: Un-spin the turret
          Transform3d dynamicTurretRotation =
              new Transform3d(
                  new Translation3d(), new Rotation3d(0, 0, -liveTurretAngle.getRadians()));

          // Field -> Camera -> Turret Center -> Robot Center
          robotPose =
              robotPose
                  .transformBy(VisionConstants.TURRET_TO_CAMERA.inverse())
                  .transformBy(dynamicTurretRotation.inverse())
                  .transformBy(VisionConstants.ROBOT_TO_TURRET.inverse());
        }

        // 3. Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > VisionConstants.MAX_AMBIGUITY)
                || robotPose.getX() < 0.0 // Must be within the field boundaries
                || robotPose.getX() > VisionConstants.FIELD_LENGTH
                || robotPose.getY() < 0.0
                || robotPose.getY() > VisionConstants.FIELD_WIDTH;

        // Add pose to log
        robotPoses.add(robotPose);
        if (rejectPose) {
          robotPosesRejected.add(robotPose);
          continue; // Skip the rest if rejected
        } else {
          robotPosesAccepted.add(robotPose);
        }

        // 4. Calculate Standard Deviations (Trust Factor)
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        int numTags = observation.tagCount();

        // Note: Unless averageTagDistance is sent from the Pi, this is likely 0.0.
        // If you add distance to your C++ array later, this logic kicks in automatically.
        double avgDist = observation.averageTagDistance();

        if (numTags > 1) {
          estStdDevs = VisionConstants.kMultiTagStdDevs;
        }

        if (numTags == 1 && avgDist > VisionConstants.MAX_AVERAGE_DISTANCE) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / VisionConstants.STD_DEV_RANGE));
        }

        curStdDevs = estStdDevs;

        // 5. Send vision observation to the Pose Estimator
        consumer.accept(robotPose.toPose2d(), observation.timestamp(), curStdDevs);
      }

      // Log camera data to AdvantageKit
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));

      if (!robotPosesAccepted.isEmpty()) {
        Logger.recordOutput(
            "Vision/Camera" + cameraIndex + "/AcceptedRobotPoses",
            robotPosesAccepted.toArray(new Pose3d[0]));
      }
      if (!robotPosesRejected.isEmpty()) {
        Logger.recordOutput(
            "Vision/Camera" + cameraIndex + "/RejectedRobotPoses",
            robotPosesRejected.toArray(new Pose3d[0]));
      }

      Logger.recordOutput("Vision/StdDevs", curStdDevs);
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}

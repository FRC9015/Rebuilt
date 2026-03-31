package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;

public class VisionIOUmbra implements VisionIO {
  private final DoubleArraySubscriber observationSub;

  public VisionIOUmbra(String cameraName) {
    // Must match "Umbra" and the camera "name" from your config.json
    var table = NetworkTableInstance.getDefault().getTable("Umbra").getSubTable(cameraName);

    // Subscribe to the "observations" topic we created in C++
    this.observationSub = table.getDoubleArrayTopic("observations").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double[] data = observationSub.get();

    // LOG THESE TO SEE WHAT IS HAPPENING
    // 1. Is the topic even found on the network?
    org.littletonrobotics.junction.Logger.recordOutput(
        "Vision/Debug/" + observationSub.getTopic().getName() + "/Exists",
        observationSub.getTopic().isValid());
    // 2. How many numbers are we getting?
    org.littletonrobotics.junction.Logger.recordOutput(
        "Vision/Debug/" + observationSub.getTopic().getName() + "/Length", data.length);

    // Actual logic
    inputs.connected = (data.length >= 8);

    if (data.length < 8) {
      inputs.poseObservations = new PoseObservation[0];
      return;
    }

    inputs.connected = true;
    double timestamp = data[0];
    Pose3d robotPose =
        new Pose3d(data[1], data[2], data[3], new Rotation3d(data[4], data[5], data[6]));
    int tagCount = (int) data[7];

    // Create the observation object for your Vision.java logic
    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              timestamp, robotPose, 0.0, // Ambiguity is filtered on the Pi, so we treat it as 0
              tagCount, 0.0, // Average distance (you could add this to C++ later)
              List.of())
        };
  }
}

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;

public class VisionIOUmbra implements VisionIO {
  private final DoubleArraySubscriber observationSub;

  /**
   * Constructor for the custom Umbra Vision coprocessor connection.
   *
   * @param cameraName The name assigned in config.json on the Pi (e.g. "starboard", "turret")
   */
  public VisionIOUmbra(String cameraName) {
    // NT4 Target: Umbra/[cameraName]/observations
    var table = NetworkTableInstance.getDefault().getTable("Umbra").getSubTable(cameraName);
    this.observationSub = table.getDoubleArrayTopic("observations").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double[] data = observationSub.get();

    // Verify array structure has the target 9 elements
    inputs.connected = (data.length >= 9);

    // If packet size is incomplete or no tags are visible, empty the observations
    if (data.length < 9 || data[7] == 0) {
      inputs.poseObservations = new PoseObservation[0];
      return;
    }

    double timestamp = data[0];

    // Calculated on the Pi side:
    //   - Represents Robot Pose if using static offsets inside C++ config.json
    //   - Represents Lens Pose if using empty offsets (0,0,0) inside C++ config.json for the turret
    // camera
    Pose3d incomingPose =
        new Pose3d(data[1], data[2], data[3], new Rotation3d(data[4], data[5], data[6]));

    int tagCount = (int) data[7];
    double averageTagDistance = data[8];

    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              timestamp,
              incomingPose,
              0.0, // Ambiguity is pre-filtered on the Pi using solvePnPGeneric
              tagCount,
              averageTagDistance,
              List.of())
        };
  }
}

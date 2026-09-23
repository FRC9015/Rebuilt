package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;

public class VisionIOUmbra implements VisionIO {
  private final DoubleArraySubscriber observationSub;
  private final String cameraName;

  public VisionIOUmbra(String cameraName) {
    this.cameraName = cameraName;
    var table = NetworkTableInstance.getDefault().getTable("Umbra").getSubTable(cameraName);
    this.observationSub = table.getDoubleArrayTopic("observations").subscribe(new double[] {});
  }

  @Override
  public String getName() {
    return this.cameraName;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double[] data = observationSub.get();
    inputs.connected = (data.length >= 9);

    if (data.length < 9 || data[7] == 0) {
      inputs.poseObservations = new PoseObservation[0];
      return;
    }

    double timestamp = data[0];
    Pose3d incomingPose =
        new Pose3d(data[1], data[2], data[3], new Rotation3d(data[4], data[5], data[6]));

    int tagCount = (int) data[7];
    double averageTagDistance = data[8];

    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              timestamp,
              incomingPose,
              0.0, // Pre-filtered on Pi
              tagCount,
              averageTagDistance,
              List.of())
        };
  }
}

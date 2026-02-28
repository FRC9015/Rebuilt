package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOSim implements VisionIO {

  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;
  private final PhotonCamera camera;

  /**
   * Creates a new VisionIOSim.
   *
   * @param name The name of the camera (must match PhotonVision config name).
   * @param robotToCamera Transform from robot center to camera.
   * @param poseSupplier Supplier for the robot pose used in simulation.
   */
  public VisionIOSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {

    this.poseSupplier = poseSupplier;

    // Initialize vision system once
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Create PhotonCamera (simulated)
    this.camera = new PhotonCamera(name);

    // Configure simulated camera properties
    SimCameraProperties cameraProperties = new SimCameraProperties();
    cameraProperties.setFPS(30);
    cameraProperties.setAvgLatencyMs(30);
    cameraProperties.setLatencyStdDevMs(5);
    cameraProperties.setCalibration(960, 720, Rotation2d.fromDegrees(90));

    // Create camera simulator
    cameraSim = new PhotonCameraSim(camera, cameraProperties);

    // Add camera to vision system
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update simulation with current robot pose
    visionSim.update(poseSupplier.get());

    var result = camera.getLatestResult();

    inputs.hasTargets = result.hasTargets();

    if (result.hasTargets()) {
      inputs.targetIDs = result.getTargets().stream().mapToInt(t -> t.getFiducialId()).toArray();
    } else {
      inputs.targetIDs = new int[0];
    }
  }
}

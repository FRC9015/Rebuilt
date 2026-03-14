package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ZoneConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ZoneLogic extends SubsystemBase {
  private Drive drive;

  public enum FieldZone {
    BLUE_ALLIANCE_LEFT,
    BLUE_ALLIANCE_RIGHT,
    RED_ALLIANCE_LEFT,
    RED_ALLIANCE_RIGHT,
    NEUTRAL_ZONE_LEFT,
    NEUTRAL_ZONE_RIGHT,
    BLUE_BOTTOM_TRENCH,
    BLUE_TOP_TRENCH,
    RED_BOTTOM_TRENCH,
    RED_TOP_TRENCH,
    UNKNOWN
  }

  public ZoneLogic(Drive drive) {
    this.drive = drive;

    // LOGS ALL THE ZONES
    Logger.recordOutput("Zones/BLUE_ALLAINCE_LEFT", ZoneConstants.BLUE_ALLIANCE_LEFT);
    Logger.recordOutput("Zones/BLUE_ALLAINCE_RIGHT", ZoneConstants.BLUE_ALLIANCE_RIGHT);
    Logger.recordOutput("Zones/RED_ALLAINCE_LEFT", ZoneConstants.RED_ALLIANCE_LEFT);
    Logger.recordOutput("Zones/RED_ALLAINCE_LEFT", ZoneConstants.RED_ALLIANCE_RIGHT);
    Logger.recordOutput("Zones/NEUTRAL_ZONE_LEFT", ZoneConstants.NEUTRAL_ZONE_LEFT);
    Logger.recordOutput("Zones/NEUTRAL_ZONE_RIGHT", ZoneConstants.NEUTRAL_ZONE_RIGHT);
    Logger.recordOutput("Zones/BLUE_BOTTOM_TRENCH", ZoneConstants.BLUE_BOTTOM_TRENCH);
    Logger.recordOutput("Zones/BLUE_TOP_TRENCH", ZoneConstants.BLUE_TOP_TRENCH);
    Logger.recordOutput("Zones/RED_BOTTOM_TRENCH", ZoneConstants.RED_BOTTOM_TRENCH);
    Logger.recordOutput("Zones/RED_TOP_TRENCH", ZoneConstants.RED_TOP_TRENCH);
  }

  public FieldZone getCurrentFieldZone(Supplier<Pose2d> pose) {
    Translation2d translationPose = pose.get().getTranslation();
    if (ZoneConstants.BLUE_ALLIANCE_LEFT.contains(translationPose)) {
      return FieldZone.BLUE_ALLIANCE_LEFT;
    }
    if (ZoneConstants.BLUE_ALLIANCE_RIGHT.contains(translationPose)) {
      return FieldZone.BLUE_ALLIANCE_RIGHT;
    }
    if (ZoneConstants.RED_ALLIANCE_LEFT.contains(translationPose)) {
      return FieldZone.RED_ALLIANCE_LEFT;
    }
    if (ZoneConstants.RED_ALLIANCE_RIGHT.contains(translationPose)) {
      return FieldZone.RED_ALLIANCE_RIGHT;
    }
    if (ZoneConstants.NEUTRAL_ZONE_LEFT.contains(translationPose)) {
      return FieldZone.NEUTRAL_ZONE_LEFT;
    }
    if (ZoneConstants.NEUTRAL_ZONE_RIGHT.contains(translationPose)) {
      return FieldZone.NEUTRAL_ZONE_RIGHT;
    }
    if (ZoneConstants.BLUE_BOTTOM_TRENCH.contains(translationPose)) {
      return FieldZone.BLUE_BOTTOM_TRENCH;
    }
    if (ZoneConstants.BLUE_TOP_TRENCH.contains(translationPose)) {
      return FieldZone.BLUE_TOP_TRENCH;
    }
    if (ZoneConstants.RED_BOTTOM_TRENCH.contains(translationPose)) {
      return FieldZone.RED_BOTTOM_TRENCH;
    }
    if (ZoneConstants.RED_TOP_TRENCH.contains(translationPose)) {
      return FieldZone.RED_TOP_TRENCH;
    }
    return FieldZone.UNKNOWN;
  }

  public Pose2d getZoneTargetPose() {
    DriverStation.Alliance alliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    FieldZone zone = getCurrentFieldZone(() -> drive.getPose());
    boolean isRed = (alliance == DriverStation.Alliance.Red);

    // Own Alliance Zone -> Aim at Hub
    if ((isRed && (zone == FieldZone.RED_ALLIANCE_LEFT || zone == FieldZone.RED_ALLIANCE_RIGHT))
        || (!isRed
            && (zone == FieldZone.BLUE_ALLIANCE_LEFT || zone == FieldZone.BLUE_ALLIANCE_RIGHT))) {
      return FieldConstants.HUB_POSE_BLUE; // TurretAngleAim handles the flip to Red Hub
    }

    if (zone == FieldZone.NEUTRAL_ZONE_LEFT
        || zone == FieldZone.BLUE_ALLIANCE_LEFT
        || zone == FieldZone.RED_ALLIANCE_LEFT) {
      // Red Robot on the physical Left side needs to hit Red-Left, but FlippingUtil means
      // we feed the "Blue-Right" pose to get "Red-Left" after rotation.
      return FieldConstants.PASSING_POSE_LEFT_BLUE;
    } else {
      return FieldConstants.PASSING_POSE_RIGHT_BLUE;
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Zones/current", getCurrentFieldZone(() -> drive.getPose()));
  }
}

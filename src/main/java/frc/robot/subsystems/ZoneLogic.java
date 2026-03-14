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
    BLUE_ALLIANCE_TOP,
    BLUE_ALLIANCE_BOTTOM,
    RED_ALLIANCE_TOP,
    RED_ALLIANCE_BOTTOM,
    NEUTRAL_ZONE_TOP,
    NEUTRAL_ZONE_BOTTOM,
    BLUE_BOTTOM_TRENCH,
    BLUE_TOP_TRENCH,
    RED_BOTTOM_TRENCH,
    RED_TOP_TRENCH,
    UNKNOWN
  }

  public ZoneLogic(Drive drive) {
    this.drive = drive;

    // LOGS ALL THE ZONES
    Logger.recordOutput("Zones/BLUE_ALLAINCE_TOP", ZoneConstants.BLUE_ALLIANCE_TOP);
    Logger.recordOutput("Zones/BLUE_ALLAINCE_BOTTOM", ZoneConstants.BLUE_ALLIANCE_BOTTOM);
    Logger.recordOutput("Zones/RED_ALLAINCE_TOP", ZoneConstants.RED_ALLIANCE_TOP);
    Logger.recordOutput("Zones/RED_ALLAINCE_TOP", ZoneConstants.RED_ALLIANCE_BOTTOM);
    Logger.recordOutput("Zones/NEUTRAL_ZONE_TOP", ZoneConstants.NEUTRAL_ZONE_TOP);
    Logger.recordOutput("Zones/NEUTRAL_ZONE_BOTTOM", ZoneConstants.NEUTRAL_ZONE_BOTTOM);
    Logger.recordOutput("Zones/BLUE_BOTTOM_TRENCH", ZoneConstants.BLUE_BOTTOM_TRENCH);
    Logger.recordOutput("Zones/BLUE_TOP_TRENCH", ZoneConstants.BLUE_TOP_TRENCH);
    Logger.recordOutput("Zones/RED_BOTTOM_TRENCH", ZoneConstants.RED_BOTTOM_TRENCH);
    Logger.recordOutput("Zones/RED_TOP_TRENCH", ZoneConstants.RED_TOP_TRENCH);
  }

  public FieldZone getCurrentFieldZone(Supplier<Pose2d> pose) {
    Translation2d translationPose = pose.get().getTranslation();
    if (ZoneConstants.BLUE_ALLIANCE_TOP.contains(translationPose)) {
      return FieldZone.BLUE_ALLIANCE_TOP;
    }
    if (ZoneConstants.BLUE_ALLIANCE_BOTTOM.contains(translationPose)) {
      return FieldZone.BLUE_ALLIANCE_BOTTOM;
    }
    if (ZoneConstants.RED_ALLIANCE_TOP.contains(translationPose)) {
      return FieldZone.RED_ALLIANCE_TOP;
    }
    if (ZoneConstants.RED_ALLIANCE_BOTTOM.contains(translationPose)) {
      return FieldZone.RED_ALLIANCE_BOTTOM;
    }
    if (ZoneConstants.NEUTRAL_ZONE_TOP.contains(translationPose)) {
      return FieldZone.NEUTRAL_ZONE_TOP;
    }
    if (ZoneConstants.NEUTRAL_ZONE_BOTTOM.contains(translationPose)) {
      return FieldZone.NEUTRAL_ZONE_BOTTOM;
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
    if ((isRed && (zone == FieldZone.RED_ALLIANCE_TOP || zone == FieldZone.RED_ALLIANCE_BOTTOM))
        || (!isRed
            && (zone == FieldZone.BLUE_ALLIANCE_TOP || zone == FieldZone.BLUE_ALLIANCE_BOTTOM))) {
      return FieldConstants.HUB_POSE_BLUE; // TurretAngleAim handles the flip to Red Hub
    }

    if (zone == FieldZone.NEUTRAL_ZONE_TOP
        || zone == FieldZone.BLUE_ALLIANCE_TOP
        || zone == FieldZone.RED_ALLIANCE_TOP) {
      // Red Robot on the physical TOP side needs to hit Red-TOP, but FlippingUtil means
      // we feed the "Blue-BOTTOM" pose to get "Red-TOP" after rotation.
      return FieldConstants.PASSING_POSE_TOP_BLUE;
    } else {
      return FieldConstants.PASSING_POSE_BOTTOM_BLUE;
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Zones/current", getCurrentFieldZone(() -> drive.getPose()));
  }
}

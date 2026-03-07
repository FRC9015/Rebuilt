package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Zones;
import frc.robot.Zones.FieldZone;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.turret.Turret;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ZoneCommands extends Command {
  private final Drive drive;
  private Supplier<Pose2d> pose;
  private final Hood hood;
  private Supplier<Boolean> run, override;
  private final DriverStation.Alliance alliance;
  private final Turret turret;
  private InterpolatingTreeMap<Double, Double> interp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  public ZoneCommands(
      Supplier<Pose2d> pose,
      Drive drive,
      Hood hood,
      Supplier<Boolean> run,
      Supplier<Boolean> override,
      Turret turret) {
    this.drive = drive;
    this.pose = pose;
    this.hood = hood;
    this.run = run;
    this.alliance = DriverStation.getAlliance().get();
    this.turret = turret;
    this.override = override;
    interp.put(0.0, 0.0);
  }

  @Override
  public void execute() {
    boolean runGet = run.get();
    boolean overrideGet = override.get();
    FieldZone currentZone = Zones.getCurrentFieldZone(pose);
    if (!overrideGet) {
      if ((currentZone == Zones.FieldZone.BLUE_BOTTOM_TRENCH_DUCK)
          || (currentZone == Zones.FieldZone.RED_BOTTOM_TRENCH_DUCK)
          || (currentZone == Zones.FieldZone.RED_TOP_TRENCH_DUCK)
          || (currentZone == Zones.FieldZone.BLUE_TOP_TRENCH_DUCK)) {
        hood.setHoodPos(0.0);
      }
      if (runGet) {
        if (currentZone == Zones.FieldZone.BLUE_ALLIANCE
            && alliance.equals(DriverStation.Alliance.Blue)) {
          new TurretAngleAim(pose, turret, FieldConstants.HUB_POSE_BLUE, drive, interp);
        } else if (currentZone == Zones.FieldZone.RED_ALLIANCE
            && alliance.equals(DriverStation.Alliance.Red)) {
          new TurretAngleAim(pose, turret, FieldConstants.HUB_POSE_BLUE, drive, interp);
        } else if (currentZone == Zones.FieldZone.NEUTRAL_ZONE_LEFT
            && alliance.equals(DriverStation.Alliance.Blue)) {
          new TurretAngleAim(pose, turret, FieldConstants.PASSING_POSE_LEFT_BLUE, drive, interp);
        } else if (currentZone == Zones.FieldZone.NEUTRAL_ZONE_RIGHT
            && alliance.equals(DriverStation.Alliance.Blue)) {
          new TurretAngleAim(pose, turret, FieldConstants.PASSING_POSE_RIGHT_BLUE, drive, interp);
        } else if (currentZone == Zones.FieldZone.NEUTRAL_ZONE_LEFT
            && alliance.equals(DriverStation.Alliance.Red)) {
          new TurretAngleAim(pose, turret, FieldConstants.PASSING_POSE_LEFT_RED, drive, interp);
        } else if (currentZone == Zones.FieldZone.NEUTRAL_ZONE_RIGHT
            && alliance.equals(DriverStation.Alliance.Red)) {
          new TurretAngleAim(pose, turret, FieldConstants.PASSING_POSE_RIGHT_RED, drive, interp);
        }
      }
    }
    Zones.logAllZones();
    Logger.recordOutput("Zones/currentZone", Zones.getCurrentFieldZone(pose));
  }
}

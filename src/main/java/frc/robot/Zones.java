package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotDimensionConstants;
import frc.robot.Constants.ZoneConstants;
import frc.robot.commands.TurretAngleAim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.turret.Turret;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Zones extends SubsystemBase {
  public static interface Zone {
    public Trigger contains(Supplier<Pose2d> pose);
  }

  public static interface PredictiveXZone extends Zone {
    public Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt);
  }

  public static class BaseZone implements Zone {
    protected final double xMin, xMax, yMin, yMax;

    public BaseZone(double xMin, double xMax, double yMin, double yMax) {
      this.xMin = xMin;
      this.xMax = xMax;
      this.yMin = yMin;
      this.yMax = yMax;
    }

    public BaseZone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
      this(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
    }

    @Override
    public Trigger contains(Supplier<Pose2d> poseSupplier) {
      return new Trigger(() -> this.containsPoint(poseSupplier.get().getTranslation()));
    }

    protected boolean containsPoint(Translation2d point) {
      return point.getX() >= xMin
          && point.getX() <= xMax
          && point.getY() >= yMin
          && point.getY() <= yMax;
    }

    public BaseZone mirroredX() {
      return new BaseZone(
          FieldConstants.FIELD_LENGTH.in(Meters) - xMax,
          FieldConstants.FIELD_LENGTH.in(Meters) - xMin,
          yMin,
          yMax);
    }

    public BaseZone mirroredY() {
      return new BaseZone(
          xMin,
          xMax,
          FieldConstants.FIELD_WIDTH.in(Meters) - yMax,
          FieldConstants.FIELD_WIDTH.in(Meters) - yMin);
    }

    /** list of corners, with the bottom left corner repeated at the end to form a closed loop */
    public Translation2d[] getCorners() {
      return new Translation2d[] {
        new Translation2d(xMin, yMin),
        new Translation2d(xMax, yMin),
        new Translation2d(xMax, yMax),
        new Translation2d(xMin, yMax),
        new Translation2d(xMin, yMin)
      };
    }
  }

  public static class PredictiveXBaseZone extends BaseZone implements PredictiveXZone {
    public PredictiveXBaseZone(double xMin, double xMax, double yMin, double yMax) {
      super(xMin, xMax, yMin, yMax);
    }

    public PredictiveXBaseZone(BaseZone baseZone) {
      super(baseZone.xMin, baseZone.xMax, baseZone.yMin, baseZone.yMax);
    }

    public PredictiveXBaseZone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
      super(xMin, xMax, yMin, yMax);
    }

    @Override
    public Trigger willContain(
        Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
      return new Trigger(
          () -> willContainPoint(pose.get().getTranslation(), fieldSpeeds.get(), dt));
    }

    protected boolean willContainPoint(Translation2d point, ChassisSpeeds fieldSpeeds, Time dt) {
      return (point.getY() >= yMin && point.getY() <= yMax)
          && ((point.getX() >= xMin && point.getX() <= xMax)
              || (point.getX() < xMin
                  && fieldSpeeds.vxMetersPerSecond * dt.in(Seconds) >= xMin - point.getX())
              || (point.getX() > xMax
                  && fieldSpeeds.vxMetersPerSecond * dt.in(Seconds) <= xMax - point.getX()));
    }

    @Override
    public PredictiveXBaseZone mirroredX() {
      return new PredictiveXBaseZone(super.mirroredX());
    }

    @Override
    public PredictiveXBaseZone mirroredY() {
      return new PredictiveXBaseZone(super.mirroredY());
    }
  }

  public static class ZoneCollection implements Zone {
    protected final Zone[] zones;

    public ZoneCollection(Zone... zones) {
      this.zones = zones;
    }

    @Override
    public Trigger contains(Supplier<Pose2d> pose) {
      Trigger combined = new Trigger(() -> false);

      for (Zone zone : zones) {
        combined = combined.or(zone.contains(pose));
      }

      return combined;
    }
  }

  public static class PredictiveXZoneCollection extends ZoneCollection implements PredictiveXZone {
    public PredictiveXZoneCollection(PredictiveXZone... zones) {
      super(zones);
    }

    @Override
    public Trigger willContain(
        Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
      Trigger combined = new Trigger(() -> false);

      for (Zone zone : zones) {
        combined = combined.or(((PredictiveXZone) zone).willContain(pose, fieldSpeeds, dt));
      }

      return combined;
    }
  }

  private static final PredictiveXBaseZone BLUE_BOTTOM_TRENCH =
      new PredictiveXBaseZone(
          FieldConstants.TRENCH_BUMP_X
              .minus(FieldConstants.TRENCH_BUMP_LENGTH.div(2))
              .minus(RobotDimensionConstants.FULL_LENGTH.div(2)),
          FieldConstants.TRENCH_BUMP_X
              .plus(FieldConstants.TRENCH_BUMP_LENGTH.div(2))
              .plus(RobotDimensionConstants.FULL_LENGTH.div(2)),
          Meters.of(0),
          FieldConstants.TRENCH_WIDTH);
  private static final PredictiveXBaseZone BLUE_TOP_TRENCH = BLUE_BOTTOM_TRENCH.mirroredY();
  private static final PredictiveXBaseZone RED_BOTTOM_TRENCH = BLUE_BOTTOM_TRENCH.mirroredX();
  private static final PredictiveXBaseZone RED_TOP_TRENCH = BLUE_TOP_TRENCH.mirroredX();

  public static final PredictiveXZoneCollection TRENCH_ZONES =
      new PredictiveXZoneCollection(
          BLUE_BOTTOM_TRENCH, BLUE_TOP_TRENCH, RED_BOTTOM_TRENCH, RED_TOP_TRENCH);

  private static final PredictiveXBaseZone BLUE_BOTTOM_TRENCH_DUCK =
      new PredictiveXBaseZone(
          FieldConstants.TRENCH_BUMP_X
              .minus(FieldConstants.TRENCH_BAR_WIDTH.div(2))
              .minus(ZoneConstants.EXTRA_DUCK_DISTANCE),
          FieldConstants.TRENCH_BUMP_X
              .plus(FieldConstants.TRENCH_BAR_WIDTH.div(2))
              .plus(ZoneConstants.EXTRA_DUCK_DISTANCE),
          Meters.of(0),
          FieldConstants.TRENCH_WIDTH);
  private static final PredictiveXBaseZone BLUE_TOP_TRENCH_DUCK =
      BLUE_BOTTOM_TRENCH_DUCK.mirroredY();
  private static final PredictiveXBaseZone RED_BOTTOM_TRENCH_DUCK =
      BLUE_BOTTOM_TRENCH_DUCK.mirroredX();
  private static final PredictiveXBaseZone RED_TOP_TRENCH_DUCK = BLUE_TOP_TRENCH_DUCK.mirroredX();

  public static final PredictiveXZoneCollection TRENCH_DUCK_ZONES =
      new PredictiveXZoneCollection(
          BLUE_BOTTOM_TRENCH_DUCK,
          BLUE_TOP_TRENCH_DUCK,
          RED_BOTTOM_TRENCH_DUCK,
          RED_TOP_TRENCH_DUCK);

  // private static final PredictiveXBaseZone BLUE_BOTTOM_BUMP =
  //     new PredictiveXBaseZone(
  //         FieldConstants.TRENCH_BUMP_X
  //             .minus(FieldConstants.TRENCH_BUMP_LENGTH.div(2))
  //             .minus(RobotDimensionConstants.FULL_LENGTH.div(2)),
  //         FieldConstants.TRENCH_BUMP_X
  //             .plus(FieldConstants.TRENCH_BUMP_LENGTH.div(2))
  //             .plus(RobotDimensionConstants.FULL_LENGTH.div(2)),
  //         FieldConstants.TRENCH_WIDTH.plus(FieldConstants.TRENCH_BLOCK_WIDTH),
  //         FieldConstants.TRENCH_WIDTH
  //             .plus(FieldConstants.TRENCH_BLOCK_WIDTH)
  //             .plus(FieldConstants.BUMP_WIDTH));
  // private static final PredictiveXBaseZone BLUE_TOP_BUMP = BLUE_BOTTOM_BUMP.mirroredY();
  // private static final PredictiveXBaseZone RED_BOTTOM_BUMP = BLUE_BOTTOM_BUMP.mirroredX();
  // private static final PredictiveXBaseZone RED_TOP_BUMP = BLUE_TOP_BUMP.mirroredX();

  // public static final PredictiveXZoneCollection BUMP_ZONES =
  //     new PredictiveXZoneCollection(BLUE_BOTTOM_BUMP, BLUE_TOP_BUMP, RED_BOTTOM_BUMP,
  // RED_TOP_BUMP);

  private static final PredictiveXBaseZone BLUE_ALLIANCE_ZONE =
      new PredictiveXBaseZone(
          Meters.of(0), FieldConstants.ALLIANCE_ZONE, Meters.of(0), FieldConstants.FIELD_WIDTH);

  private static final PredictiveXBaseZone RED_ALLIANCE_ZONE = BLUE_ALLIANCE_ZONE.mirroredX();

  private static final PredictiveXBaseZone NEUTRAL_ZONE_LEFT =
      new PredictiveXBaseZone(
          FieldConstants.ALLIANCE_ZONE.plus(FieldConstants.TRENCH_WIDTH),
          FieldConstants.FIELD_LENGTH
              .minus(FieldConstants.ALLIANCE_ZONE)
              .minus(FieldConstants.TRENCH_WIDTH),
          Meters.of(0),
          FieldConstants.FIELD_WIDTH.div(2));

  private static final PredictiveXBaseZone NEUTRAL_ZONE_RIGHT =
      new PredictiveXBaseZone(
          FieldConstants.ALLIANCE_ZONE.plus(FieldConstants.TRENCH_WIDTH),
          FieldConstants.FIELD_LENGTH
              .minus(FieldConstants.ALLIANCE_ZONE)
              .minus(FieldConstants.TRENCH_WIDTH),
          (FieldConstants.FIELD_WIDTH.div(2)),
          FieldConstants.FIELD_WIDTH);

  public static final PredictiveXZoneCollection FIELD_ALLIANCE_ZONES =
      new PredictiveXZoneCollection(BLUE_ALLIANCE_ZONE, RED_ALLIANCE_ZONE);

  // public static final PredictiveXZoneCollection FIELD_NEUTRAL_ZONE =
  //     new PredictiveXZoneCollection(NEUTRAL_ZONE);

  public enum FieldZone {
    BLUE_ALLIANCE,
    RED_ALLIANCE,
    NEUTRAL_ZONE_LEFT,
    NEUTRAL_ZONE_RIGHT,

    BLUE_BOTTOM_TRENCH,
    BLUE_TOP_TRENCH,
    RED_BOTTOM_TRENCH,
    RED_TOP_TRENCH,

    BLUE_BOTTOM_TRENCH_DUCK,
    BLUE_TOP_TRENCH_DUCK,
    RED_BOTTOM_TRENCH_DUCK,
    RED_TOP_TRENCH_DUCK,

    // BLUE_BOTTOM_BUMP,
    // BLUE_TOP_BUMP,
    // RED_BOTTOM_BUMP,
    // RED_TOP_BUMP,

    UNKNOWN
  }

  public static FieldZone getCurrentFieldZone(Supplier<Pose2d> pose) {
    // --- Trenches ---
    if (BLUE_BOTTOM_TRENCH.contains(pose).getAsBoolean()) return FieldZone.BLUE_BOTTOM_TRENCH;
    if (BLUE_TOP_TRENCH.contains(pose).getAsBoolean()) return FieldZone.BLUE_TOP_TRENCH;
    if (RED_BOTTOM_TRENCH.contains(pose).getAsBoolean()) return FieldZone.RED_BOTTOM_TRENCH;
    if (RED_TOP_TRENCH.contains(pose).getAsBoolean()) return FieldZone.RED_TOP_TRENCH;

    // --- Trench Duck ---
    if (BLUE_BOTTOM_TRENCH_DUCK.contains(pose).getAsBoolean())
      return FieldZone.BLUE_BOTTOM_TRENCH_DUCK;
    if (BLUE_TOP_TRENCH_DUCK.contains(pose).getAsBoolean()) return FieldZone.BLUE_TOP_TRENCH_DUCK;
    if (RED_BOTTOM_TRENCH_DUCK.contains(pose).getAsBoolean())
      return FieldZone.RED_BOTTOM_TRENCH_DUCK;
    if (RED_TOP_TRENCH_DUCK.contains(pose).getAsBoolean()) return FieldZone.RED_TOP_TRENCH_DUCK;

    // --- Bumps ---
    // if (BLUE_BOTTOM_BUMP.contains(pose).getAsBoolean()) return FieldZone.BLUE_BOTTOM_BUMP;
    // if (BLUE_TOP_BUMP.contains(pose).getAsBoolean()) return FieldZone.BLUE_TOP_BUMP;
    // if (RED_BOTTOM_BUMP.contains(pose).getAsBoolean()) return FieldZone.RED_BOTTOM_BUMP;
    // if (RED_TOP_BUMP.contains(pose).getAsBoolean()) return FieldZone.RED_TOP_BUMP;

    // --- Field Sections ---
    if (BLUE_ALLIANCE_ZONE.contains(pose).getAsBoolean()) return FieldZone.BLUE_ALLIANCE;
    if (RED_ALLIANCE_ZONE.contains(pose).getAsBoolean()) return FieldZone.RED_ALLIANCE;
    // if (NEUTRAL_ZONE.contains(pose).getAsBoolean()) return FieldZone.NEUTRAL;
    if (NEUTRAL_ZONE_LEFT.contains(pose).getAsBoolean()) return FieldZone.NEUTRAL_ZONE_LEFT;
    if (NEUTRAL_ZONE_LEFT.contains(pose).getAsBoolean()) return FieldZone.NEUTRAL_ZONE_RIGHT;

    return FieldZone.UNKNOWN;
  }

  public static void logAllZones() {
    Logger.recordOutput("Zones/Trenches/Blue Bottom", BLUE_BOTTOM_TRENCH.getCorners());
    Logger.recordOutput("Zones/Trenches/Blue Top", BLUE_TOP_TRENCH.getCorners());
    Logger.recordOutput("Zones/Trenches/Red Bottom", RED_BOTTOM_TRENCH.getCorners());
    Logger.recordOutput("Zones/Trenches/Red Top", RED_TOP_TRENCH.getCorners());

    Logger.recordOutput("Zones/Trenches Duck/Blue Bottom", BLUE_BOTTOM_TRENCH_DUCK.getCorners());
    Logger.recordOutput("Zones/Trenches Duck/Blue Top", BLUE_TOP_TRENCH_DUCK.getCorners());
    Logger.recordOutput("Zones/Trenches Duck/Red Bottom", RED_BOTTOM_TRENCH_DUCK.getCorners());
    Logger.recordOutput("Zones/Trenches Duck/Red Top", RED_TOP_TRENCH_DUCK.getCorners());

    // Logger.recordOutput("Zones/Bumps/Blue Bottom", BLUE_BOTTOM_BUMP.getCorners());
    // Logger.recordOutput("Zones/Bumps/Blue Top", BLUE_TOP_BUMP.getCorners());
    // Logger.recordOutput("Zones/Bumps/Red Bottom", RED_BOTTOM_BUMP.getCorners());
    // Logger.recordOutput("Zones/Bumps/Red Top", RED_TOP_BUMP.getCorners());

    Logger.recordOutput("Zones/Field/Blue Alliance", BLUE_ALLIANCE_ZONE.getCorners());
    Logger.recordOutput("Zones/Field/Red Alliance", RED_ALLIANCE_ZONE.getCorners());
    Logger.recordOutput("Zones/Field/Neutral_LEFT", NEUTRAL_ZONE_LEFT.getCorners());
    Logger.recordOutput("Zones/Field/Neutral_RIGHT", NEUTRAL_ZONE_RIGHT.getCorners());
  }

  private boolean run = true;
  private boolean override = false;

  private boolean button = true;
  private boolean the = true;

  public void setButton(boolean t) {
    button = t;
  }

  public void setthe(boolean t) {
    the = t;
  }

  public boolean getbut() {
    return button;
  }

  public boolean getthe() {
    return the;
  }

  public void toggleRun() {
    run = !run;
  }

  public boolean getRun() {
    return run;
  }

  private void setOverride(boolean value) {
    override = value;
  }

  public boolean getOverride() {
    return override;
  }

  public Command override() {
    return this.startEnd(() -> setOverride(true), () -> setOverride(false));
  }

  public Command runToggle() {
    return this.runOnce(() -> toggleRun());
  }

  private final Drive drive;
  private Supplier<Pose2d> pose;
  private final Hood hood;
  private final DriverStation.Alliance alliance;
  private final Turret turret;
  private InterpolatingTreeMap<Double, Double> interp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  public Zones(Supplier<Pose2d> pose, Drive drive, Hood hood, Turret turret) {
    this.drive = drive;
    this.pose = pose;
    this.hood = hood;
    this.alliance = DriverStation.getAlliance().get();
    this.turret = turret;
    interp.put(0.0, 0.0);
  }

  @Override
  public void periodic() {
    boolean runGet = run;
    boolean overrideGet = override;
    FieldZone currentZone = Zones.getCurrentFieldZone(pose);
    if (!overrideGet) {
      if ((currentZone == Zones.FieldZone.BLUE_BOTTOM_TRENCH_DUCK)
          || (currentZone == Zones.FieldZone.RED_BOTTOM_TRENCH_DUCK)
          || (currentZone == Zones.FieldZone.RED_TOP_TRENCH_DUCK)
          || (currentZone == Zones.FieldZone.BLUE_TOP_TRENCH_DUCK)) {
        hood.setHoodPos(0.0);
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
}

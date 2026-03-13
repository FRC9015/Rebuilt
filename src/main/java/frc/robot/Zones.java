package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotDimensionConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Zones {

  private static final double FIELD_LENGTH = FieldConstants.FIELD_LENGTH.in(Meters);
  private static final double FIELD_WIDTH = FieldConstants.FIELD_WIDTH.in(Meters);

  private static final double ALLIANCE_BLUE = FieldConstants.ALLIANCE_ZONE.in(Meters);
  private static final double ALLIANCE_RED = FIELD_LENGTH - ALLIANCE_BLUE;

  public static boolean isInRedAllianceZone(double x) {
    return x >= 0 && x < ALLIANCE_RED;
  }

  public static boolean isInBlueAllianceZone(double x) {
    return x >= ALLIANCE_BLUE && x <= FIELD_LENGTH;
  }

  public static boolean isInNeutralZone(double x) {
    return x >= ALLIANCE_RED && x < ALLIANCE_BLUE;
  }

  private static final double TRENCH_X_MIN =
      FieldConstants.TRENCH_BUMP_X
          .minus(FieldConstants.TRENCH_BUMP_LENGTH.div(2))
          .minus(RobotDimensionConstants.FULL_LENGTH.div(2))
          .in(Meters);

  private static final double TRENCH_X_MAX =
      FieldConstants.TRENCH_BUMP_X
          .plus(FieldConstants.TRENCH_BUMP_LENGTH.div(2))
          .plus(RobotDimensionConstants.FULL_LENGTH.div(2))
          .in(Meters);

  private static final double TRENCH_Y_MIN = 0;
  private static final double TRENCH_Y_MAX = FieldConstants.TRENCH_WIDTH.in(Meters);

  public static boolean isInBlueBottomTrench(double x, double y) {
    return x >= TRENCH_X_MIN && x <= TRENCH_X_MAX && y >= TRENCH_Y_MIN && y <= TRENCH_Y_MAX;
  }

  public static boolean isInBlueTopTrench(double x, double y) {
    return x >= TRENCH_X_MIN
        && x <= TRENCH_X_MAX
        && y >= FIELD_WIDTH - TRENCH_Y_MAX
        && y <= FIELD_WIDTH - TRENCH_Y_MIN;
  }

  public static boolean isInRedBottomTrench(double x, double y) {
    double min = FIELD_LENGTH - TRENCH_X_MAX;
    double max = FIELD_LENGTH - TRENCH_X_MIN;
    return x >= min && x <= max && y >= TRENCH_Y_MIN && y <= TRENCH_Y_MAX;
  }

  public static boolean isInRedTopTrench(double x, double y) {
    double min = FIELD_LENGTH - TRENCH_X_MAX;
    double max = FIELD_LENGTH - TRENCH_X_MIN;
    return x >= min
        && x <= max
        && y >= FIELD_WIDTH - TRENCH_Y_MAX
        && y <= FIELD_WIDTH - TRENCH_Y_MIN;
  }

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

  public static FieldZone getCurrentFieldZone(double x, double y) {
    if (isInBlueBottomTrench(x, y)) return FieldZone.BLUE_BOTTOM_TRENCH;
    if (isInBlueTopTrench(x, y)) return FieldZone.BLUE_TOP_TRENCH;
    if (isInRedBottomTrench(x, y)) return FieldZone.RED_BOTTOM_TRENCH;
    if (isInRedTopTrench(x, y)) return FieldZone.RED_TOP_TRENCH;

    if (isInBlueAllianceZone(x)) {
      return (y < FIELD_WIDTH / 2.0) ? FieldZone.RED_ALLIANCE_LEFT : FieldZone.RED_ALLIANCE_RIGHT;
    }
    if (isInRedAllianceZone(y)) {
      return (y < FIELD_WIDTH / 2.0) ? FieldZone.BLUE_ALLIANCE_LEFT : FieldZone.BLUE_ALLIANCE_RIGHT;
    }

    if (isInNeutralZone(y)) {
      return (y < FIELD_WIDTH / 2.0) ? FieldZone.NEUTRAL_ZONE_LEFT : FieldZone.NEUTRAL_ZONE_RIGHT;
    }

    return FieldZone.UNKNOWN;
  }

  public static FieldZone getCurrentFieldZone(Supplier<Pose2d> pose) {
    return getCurrentFieldZone(pose.get().getX(), pose.get().getY());
  }

  private static void logRectangle(String key, double xMin, double xMax, double yMin, double yMax) {
    Logger.recordOutput(
        key,
        new Pose2d[] {
          new Pose2d(xMin, yMin, new edu.wpi.first.math.geometry.Rotation2d()),
          new Pose2d(xMax, yMin, new edu.wpi.first.math.geometry.Rotation2d()),
          new Pose2d(xMax, yMax, new edu.wpi.first.math.geometry.Rotation2d()),
          new Pose2d(xMin, yMax, new edu.wpi.first.math.geometry.Rotation2d()),
          new Pose2d(xMin, yMin, new edu.wpi.first.math.geometry.Rotation2d())
        });
  }

  public static void logAllZones() {
    logRectangle(
        "Zones/Trenches/Blue Bottom", TRENCH_X_MIN, TRENCH_X_MAX, TRENCH_Y_MIN, TRENCH_Y_MAX);
    logRectangle(
        "Zones/Trenches/Blue Top",
        TRENCH_X_MIN,
        TRENCH_X_MAX,
        FIELD_WIDTH - TRENCH_Y_MAX,
        FIELD_WIDTH - TRENCH_Y_MIN);
    logRectangle(
        "Zones/Trenches/Red Bottom",
        FIELD_LENGTH - TRENCH_X_MAX,
        FIELD_LENGTH - TRENCH_X_MIN,
        TRENCH_Y_MIN,
        TRENCH_Y_MAX);
    logRectangle(
        "Zones/Trenches/Red Top",
        FIELD_LENGTH - TRENCH_X_MAX,
        FIELD_LENGTH - TRENCH_X_MIN,
        FIELD_WIDTH - TRENCH_Y_MAX,
        FIELD_WIDTH - TRENCH_Y_MIN);
    logRectangle("Zones/Field/Blue Alliance", 0, ALLIANCE_BLUE, 0, FIELD_WIDTH);
    logRectangle("Zones/Field/Red Alliance", ALLIANCE_RED, FIELD_LENGTH, 0, FIELD_WIDTH);
  }
}

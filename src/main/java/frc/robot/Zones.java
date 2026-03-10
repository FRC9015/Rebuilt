package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotDimensionConstants;

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
}

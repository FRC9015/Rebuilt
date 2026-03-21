package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class InterpTables {
  public InterpolatingTreeMap<Double, Double> timeOfFlightInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  public InterpolatingTreeMap<Double, Double> hoodAngleHubInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  public InterpolatingTreeMap<Double, Double> shooterSpeedHubInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  public InterpolatingTreeMap<Double, Double> hoodAnglePassInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  public InterpolatingTreeMap<Double, Double> shooterSpeedPassInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  public InterpTables() {

    hoodAngleHubInterp.put(2.38, 0.38);
    hoodAngleHubInterp.put(1.68, 0.115);
    hoodAngleHubInterp.put(3.31, 0.474);
    hoodAngleHubInterp.put(4.14, 0.62);
    hoodAngleHubInterp.put(3.07, 0.38);
    hoodAngleHubInterp.put(3.76, 0.4);
    hoodAngleHubInterp.put(1.37, 0.0);
    hoodAngleHubInterp.put(2.78, 0.32);
    hoodAngleHubInterp.put(2.85, 0.29);
    hoodAngleHubInterp.put(1.18, 0.0);
    hoodAngleHubInterp.put(2.01, 0.1);
    hoodAngleHubInterp.put(4.40, 0.61);
    hoodAngleHubInterp.put(3.45, 0.355);
    hoodAngleHubInterp.put(3.9, 0.4);
    hoodAngleHubInterp.put(3.1, 0.305);
    hoodAngleHubInterp.put(4.84, 0.67);
    hoodAngleHubInterp.put(5.25, 0.725);
    hoodAngleHubInterp.put(3.25, 0.385);
    hoodAngleHubInterp.put(4.4, 0.64);
    hoodAngleHubInterp.put(2.64, 0.36);

    shooterSpeedHubInterp.put(2.38, 31.0);
    shooterSpeedHubInterp.put(1.68, 32.0);
    shooterSpeedHubInterp.put(3.31, 34.0);
    shooterSpeedHubInterp.put(4.14, 37.0);
    shooterSpeedHubInterp.put(3.07, 34.0);
    shooterSpeedHubInterp.put(3.76, 37.0);
    shooterSpeedHubInterp.put(1.37, 32.0);
    shooterSpeedHubInterp.put(2.78, 33.0);
    shooterSpeedHubInterp.put(2.85, 34.0);
    shooterSpeedHubInterp.put(1.18, 31.0);
    shooterSpeedHubInterp.put(2.01, 33.0);
    shooterSpeedHubInterp.put(4.40, 37.0);
    shooterSpeedHubInterp.put(3.45, 37.0);
    shooterSpeedHubInterp.put(3.9, 37.0);
    shooterSpeedHubInterp.put(3.1, 35.0);
    shooterSpeedHubInterp.put(4.84, 38.0);
    shooterSpeedHubInterp.put(5.25, 39.0);
    shooterSpeedHubInterp.put(3.25, 34.0);
    shooterSpeedHubInterp.put(4.4, 38.0);
    shooterSpeedHubInterp.put(4.84, 38.0);
    shooterSpeedHubInterp.put(5.25, 39.0);
    shooterSpeedHubInterp.put(3.25, 34.0);
    shooterSpeedHubInterp.put(4.4, 38.0);
    shooterSpeedHubInterp.put(2.64, 33.0);

    timeOfFlightInterp.put(2.0, 1.05);
    timeOfFlightInterp.put(2.25, 1.0);
    timeOfFlightInterp.put(2.5, 1.167);
    timeOfFlightInterp.put(2.75, 1.067);
    timeOfFlightInterp.put(3.0, 1.267);
    timeOfFlightInterp.put(3.25, 1.1);
    timeOfFlightInterp.put(3.5, 1.3);
    timeOfFlightInterp.put(3.75, 1.2);
    timeOfFlightInterp.put(4.0, 1.33);
    timeOfFlightInterp.put(4.25, 0.983);
    timeOfFlightInterp.put(4.5, 1.267);
    timeOfFlightInterp.put(4.75, 1.183);
    timeOfFlightInterp.put(5.0, 1.317);

    shooterSpeedHubInterp.put(2.0, 31.0);
    shooterSpeedHubInterp.put(2.5, 34.0);
    shooterSpeedHubInterp.put(3.0, 36.0);
    shooterSpeedHubInterp.put(3.5, 37.0);
    shooterSpeedHubInterp.put(4.0, 39.0);
    shooterSpeedHubInterp.put(4.5, 38.0);
    shooterSpeedHubInterp.put(5.0, 39.0);

    hoodAngleHubInterp.put(2.0, 0.18);
    hoodAngleHubInterp.put(2.5, 0.215);
    hoodAngleHubInterp.put(3.0, 0.245);
    hoodAngleHubInterp.put(3.5, 0.285);
    hoodAngleHubInterp.put(4.0, 0.325);
    hoodAngleHubInterp.put(4.5, 0.365);
    hoodAngleHubInterp.put(5.0, 0.395);

    shooterSpeedPassInterp.put(5.43, 38.0);

    hoodAnglePassInterp.put(5.43, 0.545);

    shooterSpeedPassInterp.put(6.35, 39.0);

    hoodAnglePassInterp.put(6.35, 0.65);
    shooterSpeedPassInterp.put(5.08, 42.0);

    hoodAnglePassInterp.put(5.08, 0.4);

    shooterSpeedPassInterp.put(6.17, 44.0);

    hoodAnglePassInterp.put(6.17, 0.62);
  }
}

package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class InterpTables {
  public final InterpolatingTreeMap<Double, Double> timeOfFlightInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  public final InterpolatingTreeMap<Double, Double> hoodAngleHubInterpReal =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  public final InterpolatingTreeMap<Double, Double> shooterSpeedHubInterpReal =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  public final InterpolatingTreeMap<Double, Double> hoodAngleHubInterpSim =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  public final InterpolatingTreeMap<Double, Double> shooterSpeedHubInterpSim =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  public final InterpolatingTreeMap<Double, Double> hoodAnglePassInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  public final InterpolatingTreeMap<Double, Double> shooterSpeedPassInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  public InterpTables() {
    // Real-table hub tuning.
    hoodAngleHubInterpReal.put(2.38, 0.38);
    hoodAngleHubInterpReal.put(1.68, 0.115);
    hoodAngleHubInterpReal.put(3.31, 0.474);
    hoodAngleHubInterpReal.put(4.14, 0.62);
    hoodAngleHubInterpReal.put(3.07, 0.38);
    hoodAngleHubInterpReal.put(3.76, 0.4);
    hoodAngleHubInterpReal.put(1.37, 0.0);
    hoodAngleHubInterpReal.put(2.78, 0.32);
    hoodAngleHubInterpReal.put(2.85, 0.29);
    hoodAngleHubInterpReal.put(1.18, 0.0);
    hoodAngleHubInterpReal.put(2.01, 0.1);
    hoodAngleHubInterpReal.put(4.40, 0.61);
    hoodAngleHubInterpReal.put(3.45, 0.355);
    hoodAngleHubInterpReal.put(3.9, 0.4);
    hoodAngleHubInterpReal.put(3.1, 0.305);
    hoodAngleHubInterpReal.put(4.84, 0.67);
    hoodAngleHubInterpReal.put(5.25, 0.725);
    hoodAngleHubInterpReal.put(3.25, 0.385);
    hoodAngleHubInterpReal.put(4.4, 0.64);
    hoodAngleHubInterpReal.put(2.64, 0.36);

    shooterSpeedHubInterpReal.put(3.31, 34.0);
    shooterSpeedHubInterpReal.put(4.14, 37.0);
    shooterSpeedHubInterpReal.put(3.07, 34.0);
    shooterSpeedHubInterpReal.put(3.76, 37.0);
    shooterSpeedHubInterpReal.put(4.40, 37.0);
    shooterSpeedHubInterpReal.put(3.45, 37.0);
    shooterSpeedHubInterpReal.put(3.9, 37.0);
    shooterSpeedHubInterpReal.put(3.1, 35.0);
    shooterSpeedHubInterpReal.put(4.84, 38.0);
    shooterSpeedHubInterpReal.put(5.25, 39.0);
    shooterSpeedHubInterpReal.put(3.25, 34.0);
    shooterSpeedHubInterpReal.put(4.4, 38.0);
    shooterSpeedHubInterpReal.put(4.84, 38.0);
    shooterSpeedHubInterpReal.put(5.25, 39.0);
    shooterSpeedHubInterpReal.put(4.4, 38.0);

    shooterSpeedHubInterpReal.put(1.18, 31.0);
    shooterSpeedHubInterpReal.put(1.37, 32.0);
    shooterSpeedHubInterpReal.put(1.68, 32.0);
    shooterSpeedHubInterpReal.put(2.0, 31.0);
    shooterSpeedHubInterpReal.put(2.38, 31.0);
    shooterSpeedHubInterpReal.put(2.5, 34.0);
    shooterSpeedHubInterpReal.put(2.64, 33.0);
    shooterSpeedHubInterpReal.put(2.78, 33.0);
    shooterSpeedHubInterpReal.put(2.85, 34.0);
    shooterSpeedHubInterpReal.put(3.0, 36.0);
    shooterSpeedHubInterpReal.put(3.5, 37.0);
    shooterSpeedHubInterpReal.put(4.5, 38.0);
    shooterSpeedHubInterpReal.put(5.0, 39.0);

    // Sim table is intentionally a separate set so we can tune virtual shots
    // without disturbing the real-robot lookup table.
    hoodAngleHubInterpSim.put(1.151, 0.0);
    hoodAngleHubInterpSim.put(1.583, 0.1);
    hoodAngleHubInterpSim.put(2.055, 0.25);
    hoodAngleHubInterpSim.put(2.505, 0.3);
    hoodAngleHubInterpSim.put(3.009, 0.35);
    hoodAngleHubInterpSim.put(3.510, 0.4);
    hoodAngleHubInterpSim.put(4.092, 0.45);
    hoodAngleHubInterpSim.put(4.503, 0.5);
    hoodAngleHubInterpSim.put(5.067, 0.55);
    hoodAngleHubInterpSim.put(5.496, 0.6);

    shooterSpeedHubInterpSim.put(1.151, 32.0);
    shooterSpeedHubInterpSim.put(1.583, 32.0);
    shooterSpeedHubInterpSim.put(2.055, 32.0);
    shooterSpeedHubInterpSim.put(2.505, 35.0);
    shooterSpeedHubInterpSim.put(3.009, 36.0);
    shooterSpeedHubInterpSim.put(3.510, 38.0);
    shooterSpeedHubInterpSim.put(4.092, 40.0);
    shooterSpeedHubInterpSim.put(4.503, 41.0);
    shooterSpeedHubInterpSim.put(5.067, 42.0);
    shooterSpeedHubInterpSim.put(5.496, 43.0);

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

    shooterSpeedPassInterp.put(5.43, 38.0);
    hoodAnglePassInterp.put(5.43, 0.545);
    shooterSpeedPassInterp.put(6.35, 39.0);
    hoodAnglePassInterp.put(6.35, 0.65);
    shooterSpeedPassInterp.put(5.08, 42.0);
    hoodAnglePassInterp.put(5.08, 0.4);
    shooterSpeedPassInterp.put(6.17, 44.0);
    hoodAnglePassInterp.put(6.17, 0.62);
    shooterSpeedPassInterp.put(4.51, 36.0);
    hoodAnglePassInterp.put(4.51, 0.45);
    shooterSpeedPassInterp.put(16.0, 85.0);
    hoodAnglePassInterp.put(16.0, 1.25);
    shooterSpeedPassInterp.put(15.0, 80.0);
    hoodAnglePassInterp.put(15.0, 1.21);
    shooterSpeedPassInterp.put(14.0, 75.0);
    hoodAnglePassInterp.put(14.0, 1.16);
    shooterSpeedPassInterp.put(13.0, 70.0);
    hoodAnglePassInterp.put(13.0, 1.11);
    shooterSpeedPassInterp.put(12.0, 70.0);
    hoodAnglePassInterp.put(12.0, 1.06);
  }
}

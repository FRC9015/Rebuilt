package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class InterpTables {
  public InterpolatingTreeMap<Double, Double> timeOfFlightInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  public InterpolatingTreeMap<Double, Double> hoodAngleInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  public InterpolatingTreeMap<Double, Double> shooterSpeedInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  public InterpTables() {
    timeOfFlightInterp.put(2.38, 0.8145);
    timeOfFlightInterp.put(1.68, 0.9745);
    timeOfFlightInterp.put(3.31, 0.8845);
    timeOfFlightInterp.put(4.14, 1.0145);
    timeOfFlightInterp.put(3.07, 1.2045);
    timeOfFlightInterp.put(3.76, 1.1145);
    timeOfFlightInterp.put(1.37, 1.1245);
    timeOfFlightInterp.put(2.78, 1.3845);
    timeOfFlightInterp.put(2.85, 1.4045);
    timeOfFlightInterp.put(1.18, 1.1545);
    timeOfFlightInterp.put(2.01, 0.7945);
    timeOfFlightInterp.put(4.40, 1.1745);

    hoodAngleInterp.put(2.38, 0.38);
    hoodAngleInterp.put(1.68, 0.115);
    hoodAngleInterp.put(3.31, 0.474);
    hoodAngleInterp.put(4.14, 0.62);
    hoodAngleInterp.put(3.07, 0.38);
    hoodAngleInterp.put(3.76, 0.4);
    hoodAngleInterp.put(1.37, 0.0);
    hoodAngleInterp.put(2.78, 0.32);
    hoodAngleInterp.put(2.85, 0.29);
    hoodAngleInterp.put(1.18, 0.0);
    hoodAngleInterp.put(2.01, 0.1);
    hoodAngleInterp.put(4.40, 0.61);
    hoodAngleInterp.put(3.45, 0.355);
    hoodAngleInterp.put(3.9, 0.4);
    hoodAngleInterp.put(3.1, 0.305);
    hoodAngleInterp.put(4.84, 0.67);
    hoodAngleInterp.put(5.25, 0.725);
    hoodAngleInterp.put(3.25, 0.385);
    hoodAngleInterp.put(4.4, 0.64);
    hoodAngleInterp.put(2.64, 0.36);

    shooterSpeedInterp.put(2.38, 31.0);
    shooterSpeedInterp.put(1.68, 32.0);
    shooterSpeedInterp.put(3.31, 34.0);
    shooterSpeedInterp.put(4.14, 37.0);
    shooterSpeedInterp.put(3.07, 34.0);
    shooterSpeedInterp.put(3.76, 37.0);
    shooterSpeedInterp.put(1.37, 32.0);
    shooterSpeedInterp.put(2.78, 33.0);
    shooterSpeedInterp.put(2.85, 34.0);
    shooterSpeedInterp.put(1.18, 31.0);
    shooterSpeedInterp.put(2.01, 33.0);
    shooterSpeedInterp.put(4.40, 37.0);
    shooterSpeedInterp.put(3.45, 37.0);
    shooterSpeedInterp.put(3.9, 37.0);
    shooterSpeedInterp.put(3.1, 35.0);
    shooterSpeedInterp.put(4.84, 38.0);
    shooterSpeedInterp.put(5.25, 39.0);
    shooterSpeedInterp.put(3.25, 34.0);
    shooterSpeedInterp.put(4.4, 38.0);
    shooterSpeedInterp.put(2.64, 33.0);
  }
}

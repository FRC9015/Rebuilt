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
    timeOfFlightInterp.put(3.16, 1.1145);
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
    hoodAngleInterp.put(3.16, 0.4);
    hoodAngleInterp.put(1.37, 0.0);
    hoodAngleInterp.put(2.78, 0.32);
    hoodAngleInterp.put(2.85, 0.29);
    hoodAngleInterp.put(1.18, 0.0);
    hoodAngleInterp.put(2.01, 0.1);
    hoodAngleInterp.put(4.40, 0.61);

    shooterSpeedInterp.put(2.38, 31.0);
    shooterSpeedInterp.put(1.68, 32.0);
    shooterSpeedInterp.put(3.31, 34.0);
    shooterSpeedInterp.put(4.14, 37.0);
    shooterSpeedInterp.put(3.07, 34.0);
    shooterSpeedInterp.put(3.16, 37.0);
    shooterSpeedInterp.put(1.37, 32.0);
    shooterSpeedInterp.put(2.78, 33.0);
    shooterSpeedInterp.put(2.85, 34.0);
    shooterSpeedInterp.put(1.18, 31.0);
    shooterSpeedInterp.put(2.01, 33.0);
    shooterSpeedInterp.put(4.40, 37.0);  }
}

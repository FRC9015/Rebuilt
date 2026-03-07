package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class InterpTables {
  public InterpolatingTreeMap<Double, Double> timeOfFlightInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  public InterpolatingTreeMap<Double, Double> HoodAngleInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  public InterpolatingTreeMap<Double, Double> ShooterSpeedInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  public InterpTables() {
    timeOfFlightInterp.put(2.38, 0.0);
    timeOfFlightInterp.put(1.68, 0.0);
    timeOfFlightInterp.put(3.31, 0.0);
    timeOfFlightInterp.put(4.14, 0.0);
    timeOfFlightInterp.put(3.07, 0.0);
    timeOfFlightInterp.put(3.16, 0.0);
    timeOfFlightInterp.put(1.37, 0.0);
    timeOfFlightInterp.put(2.78, 0.0);
    timeOfFlightInterp.put(2.85, 0.0);
    timeOfFlightInterp.put(0.0, 0.0);
    timeOfFlightInterp.put(0.0, 0.0);
    timeOfFlightInterp.put(0.0, 0.0);
    
    
    
    HoodAngleInterp.put(., 0.0);

    ShooterSpeedInterp.put(0.0, 0.0);
  }
}

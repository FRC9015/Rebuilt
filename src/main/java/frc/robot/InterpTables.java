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
    
    
    
    HoodAngleInterp.put(., 0.0);

    ShooterSpeedInterp.put(0.0, 0.0);
  }
}

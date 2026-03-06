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
    
    public InterpTables(){
        timeOfFlightInterp.put(0.0,0.0);

        HoodAngleInterp.put(0.0,0.0);

        ShooterSpeedInterp.put(0.0, 0.0);
    }
}

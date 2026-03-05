package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  /** Class representing the inputs for the Shooter. */
  @AutoLog
  public static class HoodIOInputs {
    // Fields representing the hood state and inputs
    public double hoodEncoderPosition = 0.0;
    public double hoodMotorPosition = 0.0;
    public double hoodTargetPosition = 0.0;
    public boolean hoodEncoderConnected = false;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;
    public Angle launchAngle = Angle.ofBaseUnits(ShooterConstants.HOOD_RESTING_ANGLE, Degrees);
  }

  /** Method to update the inputs of the Shooter subsystem. */
  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setHoodPosition(double position) {}

  public default Angle getHoodPosition() {
    return Angle.ofBaseUnits(ShooterConstants.HOOD_RESTING_ANGLE, Degrees);
  }

  default void stopHood() {}

  default void setBrakeMode(boolean enable) {}
}

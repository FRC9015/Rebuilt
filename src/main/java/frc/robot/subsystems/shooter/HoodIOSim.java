package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ShooterConstants;

public class HoodIOSim implements HoodIO {
  private Angle launchAngle =
      Angle.ofBaseUnits(ShooterConstants.HOOD_RESTING_ANGLE, Degrees); // Default to 0 degrees

  public HoodIOSim() {}

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Simulate hood behavior here, e.g., update encoder position based on target position
    inputs.hoodEncoderPosition =
        inputs.hoodTargetPosition; // For simplicity, assume it reaches the target instantly
    inputs.hoodEncoderConnected = true; // Assume encoder is always connected in simulation
    inputs.hoodAppliedVolts = 0.0; // No voltage applied in simulation
    inputs.hoodCurrentAmps = 0.0; // No current draw in simulation
  }

  public Angle getHoodPosition() {
    return launchAngle;
  }

  public void setHoodPosition(double angle) {
    launchAngle = Angle.ofBaseUnits(Math.PI / 2 + angle, Degrees);
  }
}

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class HoodIOSim implements HoodIO {
  private double target = 0.0;
  private Angle launchAngle = Degrees.of(Constants.SimConstants.HOOD_MAX_ANGLE_DEG);

  public HoodIOSim() {}

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodTargetPosition = target;
    // Assume it reaches target position immediately in sim
    inputs.hoodEncoderPosition = target;
    inputs.hoodMotorPosition = target;
    inputs.hoodEncoderConnected = true;
    inputs.hoodAppliedVolts = 0.0;
    inputs.hoodCurrentAmps = 0.0;
    inputs.launchAngle = launchAngle;
  }

  @Override
  public Angle getHoodPosition() {
    return launchAngle;
  }

  @Override
  public void setHoodPosition(double position) {
    final double clampedPosition =
        MathUtil.clamp(position, ShooterConstants.HOOD_MIN_POS, ShooterConstants.HOOD_MAX_POS);
    target = clampedPosition;

    // Map clampedPosition (0.0 to 1.38) to (HOOD_MAX_ANGLE_DEG to HOOD_MIN_ANGLE_DEG)
    double angleDeg =
        Constants.SimConstants.HOOD_MAX_ANGLE_DEG
            - (clampedPosition / ShooterConstants.HOOD_MAX_POS)
                * (Constants.SimConstants.HOOD_MAX_ANGLE_DEG
                    - Constants.SimConstants.HOOD_MIN_ANGLE_DEG);

    launchAngle = Degrees.of(angleDeg);
  }

  @Override
  public void stopHood() {
    setHoodPosition(0.0);
  }
}

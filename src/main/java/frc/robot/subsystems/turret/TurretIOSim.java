package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.TurretConstants;

public class TurretIOSim implements TurretIO {
  private double appliedTurretRotation = 0.0;
  private double setpointDegrees = 0.0;

  public TurretIOSim() {}

  public TurretIOInputs inputs = new TurretIOInputs();

  public double getAppliedTurretRotation() {
    return appliedTurretRotation;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretSetpoint = setpointDegrees;
    // Assume we instantly reach the setpoint in simulation
    inputs.turretResolvedPosition = setpointDegrees / 360.0;
    inputs.turretResolvedPositionDegrees = setpointDegrees;
    inputs.turretAtSetpoint = true;
    this.inputs = inputs;
  }

  @Override
  public void stop() {}

  @Override
  public void setBrakeMode() {}

  @Override
  public void setCoastMode() {}

  @Override
  public void setTurretPosition(double positionDegrees) {
    double rotations = positionDegrees / 360.0;
    double safePosition =
        MathUtil.clamp(
            rotations, TurretConstants.MINROTATION + 0.05, TurretConstants.MAXROTATION - 0.05);
    setpointDegrees = safePosition * 360.0;
  }

  @Override
  public void seedMotorPosition(double positionRotations) {}

  @Override
  public void setTurretSetPoint(double value) {
    setpointDegrees = value;
  }
}

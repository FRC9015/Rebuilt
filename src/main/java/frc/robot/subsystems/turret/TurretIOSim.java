package frc.robot.subsystems.turret;

public class TurretIOSim implements TurretIO {
  private double appliedTurretRotation = 0.0;

  public TurretIOSim() {}

  public TurretIOInputs inputs = new TurretIOInputs();

  public double getAppliedTurretRotation() {
    return appliedTurretRotation;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    this.inputs = inputs;
  }

  @Override
  public void stop() {}

  @Override
  public void setBrakeMode() {}

  @Override
  public void setCoastMode() {}

  @Override
  public void setTurretPosition(double value) {
    inputs.turretResolvedPosition = (value);
  }

  @Override
  public void seedMotorPosition(double positionRotations) {}

  @Override
  public void setTurretSetPoint(double value) {
    inputs.turretSetpoint = value;
  }
}

package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
  private double targetSpeed = 0.0;
  private double kickerSpeed = 0.0;

  public ShooterIOSim() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.flywheelTargetSpeed = targetSpeed;
    // Assume it reaches target speed immediately in sim
    inputs.flywheelCurrentSpeed = targetSpeed;
    inputs.flywheelRPM = targetSpeed * 60.0; // speed is RPS, RPM = RPS * 60
    inputs.flywheelLinearVelocity =
        Units.MetersPerSecond.of(
            2 * Math.PI * Constants.SimConstants.FLYWHEEL_RADIUS_METERS * targetSpeed);

    if (Math.abs(inputs.flywheelCurrentSpeed - inputs.flywheelTargetSpeed)
            < Constants.ShooterConstants.FLYWHEEL_RPM_TOLERANCE
        && inputs.flywheelTargetSpeed != 0.0) {
      inputs.flywheelAtSpeed = true;
    } else {
      inputs.flywheelAtSpeed = false;
    }

    inputs.kickerRPM = kickerSpeed * 60.0;
  }

  @Override
  public void setFlyWheelSpeed(double speed) {
    targetSpeed = speed;
  }

  @Override
  public void setKickerSpeed(double speed) {
    kickerSpeed = speed;
  }

  @Override
  public double getFlyWheelSpeed() {
    return targetSpeed;
  }

  @Override
  public void stopFlywheels() {
    targetSpeed = 0.0;
  }

  @Override
  public void stopKicker() {
    kickerSpeed = 0.0;
  }

  @Override
  public void stopShooter() {
    stopFlywheels();
    stopKicker();
  }
}

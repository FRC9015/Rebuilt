package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  /** Class representing the inputs for the Shooter. */
  @AutoLog
  public static class ShooterIOInputs {

    // Fields representing the flywheel state and inputs
    public double flywheelAppliedVolts = 0.0;
    public double flywheelCurrentAmps = 0.0;
    public double flywheelCurrentSpeed = 0.0;
    public double flywheelRPM = 0.0;

    // Fields representing the hood state and inputs
    public double hoodEncoderPosition = 0.0;
    public double hoodTargetPosition = 0.0;
    public boolean hoodEncoderConnected = false;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;
  }

  /** Method to update the inputs of the Shooter subsystem. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setFlyWheelSpeed(double speed) {}

  public default double getFlyWheelSpeed() {
    return 0.0;
  }

  public default void setHoodPosition(double position) {}

  public default double getHoodPosition() {
    return 0.0;
  }

  default void stopFlywheels() {}

  default void stopHood() {}

  default void stopShooter() {}

  default void setBrakeMode(boolean enable) {}
}

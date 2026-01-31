package frc.robot.subsystems.shooter;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  /** Class representing the inputs for the Shooter. */
  @AutoLog
  public static class ShooterIOInputs {

    // Fields representing the shooter state and inputs
    public double ShooterAppliedVolts = 0.0;
    public double ShooterCurrentAmps = 0.0;
    public double ShooterCurrentSpeed = 0.0;
    public boolean ShooterEncoderConnected = false;
    public double ShooterEncoderPosition = 0.0;
    public double ShooterTargetPosition = 0.0;
    public double ShooterRPM = 0.0;
  }

  /** Method to update the inputs of the Shooter subsystem. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterSpeed(double speed) {}

  public default void setIntakePosition(double position) {}

  public default double getVelocity() {
    return 0.0;
  }

  default void stop() {}

  default void setBrakeMode(boolean enable) {}
}

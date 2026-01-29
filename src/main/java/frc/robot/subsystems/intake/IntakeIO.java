package frc.robot.subsystems.intake;

import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  /** Class representing the inputs for the Intake. */
  @AutoLog
  public static class IntakeIOInputs {

    public enum IntakePositions {
      STOWED(Constants.intakeConstants.INTAKE_STOWED_POSITION),
      DEPLOYED(Constants.intakeConstants.INTAKE_DEPLOYED_POSITION);

      private final double position;

      private IntakePositions(double position) {
        this.position = position;
      }

      public double getIntakePosition() {
        return position;
      }
    }

    // Fields representing the intake state and inputs
    public double IntakeAppliedVolts = 0.0;
    public double IntakeCurrentAmps = 0.0;
    public double IntakeCurrentSpeed = 0.0;
    public boolean IntakeEncoderConnected = false;
    public double IntakeEncoderPosition = 0.0;
    public double IntakeTargetPosition = 0.0;
    public double IntakeRPM = 0.0;
  }

  /** Method to update the inputs of the Intake subsystem. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeSpeed(double speed) {}

  public default void setIntakePosition(double position) {}

  public default double getVelocity() {
    return 0.0;
  }

  public default double getVelocity() {
    return 0.0;
  }

  default void stop() {}

  default void setBrakeMode(boolean enable) {}
}

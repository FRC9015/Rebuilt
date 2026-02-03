package frc.robot.subsystems.intake;

import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  /** Container for intake inputs used for logging and control. */
  @AutoLog
  public static class IntakeIOInputs {

    public enum IntakePositions {
      STOWED(Constants.IntakeConstants.INTAKE_STOWED_POSITION),
      DEPLOYED(Constants.IntakeConstants.INTAKE_DEPLOYED_POSITION);

      private final double position;

      private IntakePositions(double position) {
        this.position = position;
      }

      public double getIntakePosition() {
        return position;
      }
    }

    // Fields representing the intake state and inputs
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;
    public double intakeCurrentSpeed = 0.0;
    public boolean intakeEncoderConnected = false;
    public double intakeEncoderPosition = 0.0;
    public double intakeTargetPosition = 0.0;
    public double intakeRPM = 0.0;
  }

  /** Method to update the inputs of the Intake subsystem. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeSpeed(double speed) {}

  public default void setIntakePosition(double position) {}

  public default void updatePIDFromDashboard() {}

  public default double getVelocity() {
    return 0.0;
  }

  default void stop() {}

  default void setBrakeMode(boolean enable) {}
}

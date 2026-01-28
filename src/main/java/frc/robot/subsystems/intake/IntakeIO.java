package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.Alert;

public interface IntakeIO {

  /** Class representing the inputs for the Intake. */
  @AutoLog
  public static class IntakeIOInputs {

    // Fields representing the intake state and inputs
    public double IntakeAppliedVolts = 0.0;
    public double IntakeCurrentAmps = 0.0;
    public boolean IntakeEncoderConnected = false;
    public double IntakeEncoderPosition = 0.0;
    public double IntakeRPM = 0.0;
    public Alert intakeEncoderDisconnectedAlert = new Alert("Intake encoder disconnected", Alert.AlertType.kWarning);
  }

  /** Method to update the inputs of the Intake subsystem. */
  public default void updateInputs(IntakeIOInputs inputs) {

  }

  public default void setIntakeSpeed(double rpm) {

  
  }

  public default void setIntakePosition(double position) {

  }

  public default double getVelocity() {
    return 0.0;
  }

  default void stop() {}

  default void setBrakeMode(boolean enable) {}


}

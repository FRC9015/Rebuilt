package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  /** Class representing the inputs for the Intake. */
  @AutoLog
  public static class IntakeIOInputs {

    // Fields representing the intake state and inputs
    public double IntakeAppliedVolts = 0.0;
    public double IntakeCurrentAmps = 0.0;
    public boolean IntakeEncoderConnected = false;
    public double IntakeEncoderValue = 0.0;
    public double IntakeRPM = 0.0;
  }

  /** Method to update the inputs of the Intake subsystem. */
  public default void updateInputs(IntakeIOInputs inputs) {

  }

  public default void setIntakeSpeed(double rpm) {

  
  }

  public default void setIntakePosition(double position) {

  }

  default void stop() {}

  default void setBrakeMode(boolean enable) {}


}

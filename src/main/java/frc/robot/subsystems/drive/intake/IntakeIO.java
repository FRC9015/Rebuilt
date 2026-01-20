package frc.robot.subsystems.drive.intake;

import org.littletonrobotics.junction.AutoLog;



public class IntakeIO {
    
/** Class representing the inputs for the Intake. */
  @AutoLog
  public static class IntakeIOInputs {

    // Fields representing the intake state and inputs
    public double IntakeAppliedVolts = 0.0;
    public double IntakeCurrentAmps = 0.0;
    public boolean IntakeEncoderConnected = false;
    public double IntakeRPM = 0.0;
  }

    public void updateInputs(IntakeIOInputs inputs) {}

}


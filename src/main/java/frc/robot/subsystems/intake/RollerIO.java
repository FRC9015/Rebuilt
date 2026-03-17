package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

  /** Class representing the inputs for the intake rollers. */
  @AutoLog
  public static class RollerIOInputs {
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
    public double rollerCurrentSpeed = 0.0;
    public int fuelInside = 0;
  }

  /** Method to update the inputs of the Intake subsystem. */
  public default void updateInputs(RollerIOInputs inputs) {}

  public default void setRollerSpeed(double speed) {}

  public default void setRollerSpeed(boolean runIntake) {}

  public default void setRollerVolts(double voltage) {}

  default void stop() {}

  default void setBrakeMode(boolean enable) {}
}

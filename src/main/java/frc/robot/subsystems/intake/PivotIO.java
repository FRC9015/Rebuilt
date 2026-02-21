package frc.robot.subsystems.intake;

import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  /** Class representing the inputs for the intake pivot. */
  @AutoLog
  public static class PivotIOInputs {
    public double pivotApppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;
    public double pivotCurrentSpeed = 0.0;
    public double pivotPosition = 0.0;
  }

  public enum PivotPositions {
    STOWED(Constants.intakeConstants.INTAKE_STOWED_POSITION),
    DEPLOYED(Constants.intakeConstants.INTAKE_DEPLOYED_POSITION);

    private final double position;

    private PivotPositions(double position) {
      this.position = position;
    }

    public double getPivotPosition() {
      return position;
    }
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setPivotPosition(double position) {}

  public default void setPivotPosition(PivotPositions position) {}

  public default void updatePIDFromDashboard() {}

  default void stop() {}

  default void setBrakeMode(boolean enable) {}
}

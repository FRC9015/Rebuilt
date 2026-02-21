package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/** I/O interface for the climb subsystem. */
public interface ClimbIO {

  @AutoLog
  /** Inputs produced by the climb hardware layer (auto-logged). */
  public static class ClimbIOInputs {
    /** Preset climb positions with associated encoder setpoints. */
    public static enum ClimbPositions {
      ReadyToLatch(0),
      ReadyToClimbL1(0.1),
      ReadyToClimbL2(0.2),
      ReadyToClimbL3(0.3),
      FullyClimbedL1(2),
      FullyClimbedL2(3),
      FullyClimbedL3(4),
      Retracted(0);

      private final double climbEncoderPositions;

      private ClimbPositions(double climbEncoderPositions) {
        this.climbEncoderPositions = climbEncoderPositions;
      }

      public double getClimbEncoderPositions() {
        return climbEncoderPositions;
      }
    }

    public double climberAppliedVolts = 0.0;
    public double climberCurrentAmps = 0.0;
    public double climberRPM = 0.0;
    public double climberPosition = 0.0;
    public double servoPosition = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Stop the Climb. */
  default void stop() {}

  /** Enable or disable brake mode on the climb motor. */
  default void setBrakeMode(boolean enable) {}

  /** Sets the voltage for the climb motor. */
  default void setClimbVoltage(double voltage) {}

  /** Sets target posistion */
  default void setClimbPosition(double position) {}
}

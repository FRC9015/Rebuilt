package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;
import org.littletonrobotics.junction.Logger;

/** Subsystem for controlling the robot climb mechanism. */
public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  /**
   * Constructs an Climb subsystem.
   *
   * @param io The input/output interface for the Climb.
   */
  public Climb(ClimbIO io) {
    this.io = io;
    this.setDefaultCommand(run(() -> zeroClimbDefault()));
  }

  /**
   * Sets target position for the climber
   *
   * @param position pre-set climb positions listed in ClimbIO
   */
  public void setPresetPosition(ClimbIOInputs.ClimbPositions position) {
    // TODO: add safety checks to make sure you don't go past maxPosition.
    io.setClimbPosition(position);
  }

  public Command setClimbPreset(ClimbIOInputs.ClimbPositions position) {
    return this.startEnd(() -> setPresetPosition(position), () -> io.stop());
  }

  public void zeroClimbDefault() {
    if (!inputs.climbZeroed) {
      io.zeroClimb();
    }
    else {
      new InstantCommand();
    }
  }

  public boolean readyToClimbL1 () {
    return inputs.climberPosition != 0.1;
  }
  
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }
}

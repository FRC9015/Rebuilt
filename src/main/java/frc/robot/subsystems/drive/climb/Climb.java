package frc.robot.subsystems.drive.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.climb.ClimbIO.ClimbIOInputs;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private final PIDController pidController;

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kTolerance = 0.0;

  public Climb(ClimbIO io) {
    this.io = io;
    this.pidController = new PIDController(kP, kI, kD);
    pidController.setTolerance(kTolerance);
  }

  public void setPresetPosition(ClimbIOInputs.ClimbPositions position) {
    pidController.setSetpoint(position.getClimbEncoderPositions());
    double output = pidController.calculate(inputs.climberPosition);
    io.setClimbRPM(output);

    Logger.recordOutput("Climber/Setpoint", position.getClimbEncoderPositions());
    Logger.recordOutput("Climber/Output", output);
    Logger.recordOutput("Climber/PositionInput", inputs.climberPosition);    
  } 
}

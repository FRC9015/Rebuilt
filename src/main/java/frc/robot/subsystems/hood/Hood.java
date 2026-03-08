package frc.robot.subsystems.hood;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double setpoint = 0.0;

  private final HoodIO io;

  public Hood(HoodIO io) {
    this.io = io;
  }

  public Command stopHood() {
    return this.run(() -> io.stopHood());
  }

  public Command setHoodPosition(double position) {
    return this.run(() -> io.setHoodPosition(position));
  }

  public void setHoodPos(double value) {
    io.setHoodPosition(value);
  }

  public Command incrementhoodCommand(double value) {
    return runOnce(() -> incrementHoodAngle(value));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // io.setHoodPosition(this.setpoint);
    Logger.processInputs("Hood", inputs);
    Logger.recordOutput("Hood/setpoint", setpoint);
  }

  public Angle getLaunchAngle() {
    return inputs.launchAngle;
  }

  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public void incrementHoodAngle(double value) {
    this.setpoint += (0.005 * value);
  }

  public double returnHoodSetpoint() {
    return inputs.hoodTargetPosition;
  }
}

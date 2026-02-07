package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Subsystem that controls the robot's intake mechanism (roller + pivot). */
public class Intake extends SubsystemBase {
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO io;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setIntakeSpeed(double speedValue) {
    io.setIntakeSpeed(speedValue);
  }

  public void setIntakeReverseSpeed(double speedValue) {

    io.setIntakeSpeed(-speedValue);
  }

  public void setPivotSpeed(double speedValue) {
    io.setIntakePosition(speedValue);
  }

  public Command runIntakeAtSpeed(double intakeSpeed) {
    Logger.recordOutput("Intake/Speed", intakeSpeed);

    return this.startEnd(() -> this.setIntakeSpeed(intakeSpeed), () -> io.stop());
  }

  public Command runIntakeAtReverseSpeed(double speed) {
    Logger.recordOutput("Intake/Speed", speed);
    return this.startEnd(() -> this.setIntakeReverseSpeed(speed), () -> io.stop());
  }

  public Command stopIntake() {
    return this.run(() -> io.stop());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}

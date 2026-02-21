package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final RollerIO roller;
  private final PivotIO pivot;

  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  private PIDController pivotPIDController;

  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;
  private double toleranceMeters = 0.0;
  private double idleSpeed = 0.0;

  public Intake(RollerIO roller, PivotIO pivot) {
    this.roller = roller;
    this.pivot = pivot;

    pivotPIDController = new PIDController(kP, kI, kD);
    pivotPIDController.setTolerance(toleranceMeters);
  }

  // Minimum Value of speedValue: -512.0
  // Maximum Value of speedValkue: 511.998046875

  public void setIntakeSpeed(double speedValue) {
    roller.setRollerSpeed(speedValue);
  }

  public void setIntakeReverseSpeed(double speedValue) {
    roller.setRollerSpeed(-speedValue);
  }

  public void setPivotPosition(double position) {
    pivot.setPivotPosition(position);

    Logger.recordOutput("Pivot/setpoint", position);
  }

  public Command setPivotPosition(PivotIO.PivotPositions position) {
    return this.run(() -> setPivotPosition(position.getPivotPosition()));
  }

  public Command runIntakeAtSpeed(double intakeSpeed, double pivotPosition) {

    return this.startEnd(
        () -> {
          this.setIntakeSpeed(intakeSpeed);
          this.setPivotPosition(pivotPosition);
        },
        () -> {
          this.setIntakeSpeed(idleSpeed);
          this.setPivotPosition(pivotPosition);
        });
  }

  public Command runIntakeAtReverseSpeed(double speed) {
    return this.startEnd(
        () -> this.setIntakeReverseSpeed(speed), () -> this.setIntakeReverseSpeed(idleSpeed));
  }

  public Command stopIntake() {
    return this.run(() -> roller.stop());
  }

  @Override
  public void periodic() {
    roller.updateInputs(rollerInputs);
    pivot.updateInputs(pivotInputs);

    Logger.processInputs("Intake/Roller", rollerInputs);
    Logger.processInputs("Intake/Pivot", pivotInputs);
  }
}

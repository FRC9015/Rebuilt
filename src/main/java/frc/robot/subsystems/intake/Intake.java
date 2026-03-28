package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final RollerIO roller;
  private final PivotIO pivot;

  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  public Intake(RollerIO roller, PivotIO pivot) {
    this.roller = roller;
    this.pivot = pivot;
  }

  public void setRollerSpeed(double speedValue) {
    roller.setRollerSpeed(speedValue);
  }

  public void setRollerVoltage(double voltage) {
    roller.setRollerVolts(voltage);
  }

  public void setRollerReverseSpeed(double speedValue) {
    roller.setRollerSpeed(-speedValue);
  }

  public void setPivotPosition(double position) {
    pivot.setPivotPosition(position);

    Logger.recordOutput("Pivot/setpoint", position);
  }

  public Command setPivotPosition(PivotIO.PivotPositions position) {
    return this.run(() -> setPivotPosition(position.getPivotPosition()));
  }

  public Command runIntakeAtSpeed(double intakeSpeed, PivotIO.PivotPositions pivotPosition) {

    return this.run(
        () -> {
          this.setRollerSpeed(intakeSpeed);
          this.setPivotPosition(pivotPosition.getPivotPosition());
        });
  }

  public Command runRollerAtSpeed(double speed) {
    return this.startEnd(() -> this.setRollerSpeed(speed), () -> roller.stop());
  }

  public Command runRollerAtVoltage(double voltage) {
    return this.startEnd(() -> this.setRollerVoltage(voltage), () -> roller.stop());
  }

  public Command stopRoller() {
    return this.run(() -> roller.stop());
  }

  public Command runIntakeSim() {
    return this.startEnd(() -> roller.setRollerSpeed(true), () -> roller.setRollerSpeed(false));
  }

  public Command setIntakeVolts(double volts) {
    return this.startEnd(() -> pivot.setVolts(volts), () -> pivot.setVolts(0));
  }

  // Only used for simulation
  public boolean isFuelInsideIntake() {
    return rollerInputs.fuelInside > 0;
  }

  public int getIntakeFuelCount() {
    return rollerInputs.fuelInside;
  }

  public Command agitateIntakeCommand() {
    return new SequentialCommandGroup(
            this.run(
                    () -> {
                      setPivotPosition(PivotIO.PivotPositions.AGITATE.getPivotPosition());
                      setRollerSpeed(50);
                    })
                .withTimeout(0.2),
            this.run(
                    () -> {
                      setPivotPosition(PivotIO.PivotPositions.AGITATE_MIDDLE.getPivotPosition());
                      setRollerSpeed(50);
                    })
                .withTimeout(0.2))
        .repeatedly()
        .finallyDo(
            (interrupted) -> {
              setPivotPosition(PivotIO.PivotPositions.DEPLOYED).withTimeout(0.5);
              this.setRollerVoltage(0);
            });
  }

  @Override
  public void periodic() {
    roller.updateInputs(rollerInputs);
    pivot.updateInputs(pivotInputs);

    Logger.processInputs("Intake/Roller", rollerInputs);
    Logger.processInputs("Intake/Pivot", pivotInputs);
  }
}

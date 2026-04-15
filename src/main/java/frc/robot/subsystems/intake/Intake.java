package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final RollerIO roller;
  private final PivotIO pivot;
  private final Alert jamAlert;

  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  private static final double jamCurrentAmps = 30.0;
  private static final double jamRPMThreshold = 50.0;
  private static final int jamCyclesThreshold = 10; // ~0.2s at 50Hz

  private static boolean leftHighCurrent;
  private static boolean leftLowSpeed;
  private static boolean leftIsRunning;
  private static boolean rightHighCurrent;
  private static boolean rightLowSpeed;
  private static boolean rightIsRunning;

  @AutoLogOutput private int jamCycles = 0;

  public Intake(RollerIO roller, PivotIO pivot) {
    this.roller = roller;
    this.pivot = pivot;
    jamAlert = new Alert("Intake roller jam detected!", AlertType.kInfo);
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

  public void stopTheRollers() {
    roller.stop();
  }

  public Command setPivotPosition(PivotIO.PivotPositions position) {
    return this.startEnd(() -> setPivotPosition(position.getPivotPosition()), () -> stopPivot());
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

  /**
   * Reverses intake rollers to clear a detected jam.
   *
   * @return A command that reverses rollers briefly, then stops.
   */
  public Command reverseRollersWhenJammed() {
    return this.runEnd(() -> this.setRollerReverseSpeed(34), this::stopTheRollers).withTimeout(0.5);
  }

  /**
   * Runs intake rollers and automatically reverses when a jam is detected.
   *
   * @param speed Speed provided to intake rollers.
   * @return A command that runs rollers with automatic unjam behavior.
   */
  public Command runRollerWithAutoUnjam(double speed) {
    return runRollerAtSpeed(speed)
        .until(this::isJamDetected)
        .andThen(reverseRollersWhenJammed())
        .repeatedly()
        .finallyDo(interrupted -> this.stopRoller());
  }

  public Command runRollerAtVoltage(double voltage) {
    return this.startEnd(() -> this.setRollerVoltage(voltage), () -> roller.stop());
  }

  public Command stopRoller() {
    return this.run(() -> roller.stop());
  }

  /**
   * @return True if one or more intake rollers are stalled for the jam threshold.
   */
  public boolean isJamDetected() {
    return jamCycles >= jamCyclesThreshold;
  }

  public void stopPivot() {
    pivot.stop();
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
                      setRollerSpeed(-20);
                    })
                .withTimeout(0.5),
            this.run(
                    () -> {
                      setPivotPosition(PivotIO.PivotPositions.AGITATE_MIDDLE.getPivotPosition());
                      setRollerSpeed(-20);
                    })
                .withTimeout(0.5))
        .repeatedly()
        .finallyDo(
            (interrupted) -> {
              setPivotPosition(PivotIO.PivotPositions.DEPLOYED).withTimeout(0.5);
              this.stopTheRollers();
            });
  }

  public Command pullIntakeCommand() {
    return new SequentialCommandGroup(
            this.run(
                () -> {
                  setPivotPosition(PivotIO.PivotPositions.STOWED.getPivotPosition());
                  setRollerSpeed(-30);
                }))
        .finallyDo(
            (interrupted) -> {
              setPivotPosition(PivotIO.PivotPositions.DEPLOYED).withTimeout(0.5);
              this.stopTheRollers();
            });
  }

  @Override
  public void periodic() {
    roller.updateInputs(rollerInputs);
    pivot.updateInputs(pivotInputs);

    leftHighCurrent = rollerInputs.rollerLeftCurentAmps >= jamCurrentAmps;
    leftLowSpeed = Math.abs(rollerInputs.rollerLeftCurrentSpeed) <= jamRPMThreshold;
    leftIsRunning = Math.abs(rollerInputs.rollerLeftAppliedVolts) > 0.1;

    rightHighCurrent = rollerInputs.rollerRightCurentAmps >= jamCurrentAmps;
    rightLowSpeed = Math.abs(rollerInputs.rollerRightCurrentSpeed) <= jamRPMThreshold;
    rightIsRunning = Math.abs(rollerInputs.rollerRightAppliedVolts) > 0.1;

    boolean leftStalled = leftHighCurrent && leftLowSpeed && leftIsRunning;
    boolean rightStalled = rightHighCurrent && rightLowSpeed && rightIsRunning;

    if (leftStalled || rightStalled) {
      jamCycles++;
    } else {
      jamCycles = 0;
    }

    jamAlert.set(isJamDetected());

    Logger.processInputs("Intake/Roller", rollerInputs);
    Logger.processInputs("Intake/Pivot", pivotInputs);
  }
}

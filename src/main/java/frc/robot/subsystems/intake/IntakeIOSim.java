package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import frc.robot.Constants.SimConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/** Simulation implementation of {@link IntakeIO}. */
public class IntakeIOSim implements IntakeIO {
  private final IntakeSimulation intakeSimulation;

  public IntakeIOSim(SwerveDriveSimulation driveTrainSimulation) {
    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            SimConstants.GAMEPIECE,
            // Specify the drivetrain to which this intake is attached
            driveTrainSimulation,
            // Width of the intake
            Meters.of(SimConstants.INTAKE_WIDTH),
            // The extension length of the intake beyond the robot's frame (when activated)
            Meters.of(SimConstants.INTAKE_LENGTH),
            // The intake is mounted on the back side of the chassis
            IntakeSimulation.IntakeSide.FRONT,
            // The intake can hold up to 50 Fuel
            SimConstants.HOPPER_CAPACITY);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.fuelInside = intakeSimulation.getGamePiecesAmount();
  }

  @Override
  public void setRunning(boolean runIntake) {
    if (runIntake) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
  }

  @Override
  public boolean isFuelInsideIntake() {
    return intakeSimulation.getGamePiecesAmount() > 0;
  }
}

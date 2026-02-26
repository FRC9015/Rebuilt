package frc.robot.commands;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;

public class ShootAtAngleSim extends Command {
  private final IntakeSimulation simIntake;
  private final SwerveDriveSimulation simDrive;
  private final Hood hoodSim;
  private final Shooter shooterSim;

  public ShootAtAngleSim(IntakeSimulation simIntake, SwerveDriveSimulation simDrive, Hood hoodSim, Shooter shooterSim) {
    this.simIntake = simIntake;
    this.simDrive = simDrive;
    this.hoodSim = hoodSim;
    this.shooterSim = shooterSim;
    addRequirements(IntakeSimulation);
  }  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}

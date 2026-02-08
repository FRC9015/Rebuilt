// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CameraConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXMapleSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSparkFlex;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkFlex;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Indexer indexer;
  private SwerveDriveSimulation simDrive = null;
  private final Intake intake;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private double intakeRollerValue = 0; // TODO FIX THESE NUMBERS
  private double indexerRollerValue = 0;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision("placeholder", CameraConstants.placeHolderCamera));
        indexer = new Indexer(new IndexerIOSparkFlex(Constants.MotorIDConstants.INDEXER_MOTOR_ID));
        intake =
            new Intake(
                new IntakeIOSparkFlex(
                    Constants.IntakeConstants.INTAKE_MOTOR_ID,
                    Constants.IntakeConstants.INTAKE2_MOTOR_ID));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        simDrive =
            new SwerveDriveSimulation(
                Drive.mapleSimConfig, new Pose2d(new Translation2d(3, 3), new Rotation2d()));

        SimulatedArena.getInstance().addDriveTrainSimulation(simDrive);
        drive =
            new Drive(
                new GyroIOSim(simDrive.getGyroSimulation()),
                new ModuleIOTalonFXMapleSim(TunerConstants.FrontLeft, simDrive.getModules()[0]),
                new ModuleIOTalonFXMapleSim(TunerConstants.FrontRight, simDrive.getModules()[1]),
                new ModuleIOTalonFXMapleSim(TunerConstants.BackLeft, simDrive.getModules()[2]),
                new ModuleIOTalonFXMapleSim(TunerConstants.BackRight, simDrive.getModules()[3]));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision("placeholder", CameraConstants.placeHolderCamera));
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        indexer = new Indexer(new IndexerIO() {});
        intake = new Intake(new IntakeIOSim());
        break;

      case REPLAY:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision("placeholder", CameraConstants.placeHolderCamera));
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        break;

      default:
        throw new IllegalStateException("Unexpected value: " + Constants.currentMode);
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    driverController.leftBumper().whileTrue(intake.runIntakeAtSpeed(intakeRollerValue));
    driverController.rightBumper().whileTrue(indexer.runIndexer(indexerRollerValue));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput("FieldSimulation/RobotPosition", simDrive.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }
}

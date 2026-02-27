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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TurretAngleAim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs.ClimbPositions;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXMapleSim;
import frc.robot.subsystems.gamestate.GameState;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSparkFlex;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.PivotIO;
import frc.robot.subsystems.intake.PivotIO.PivotPositions;
import frc.robot.subsystems.intake.PivotIOSim;
import frc.robot.subsystems.intake.PivotIOTalonFX;
import frc.robot.subsystems.intake.RollerIO;
import frc.robot.subsystems.intake.RollerIOSim;
import frc.robot.subsystems.intake.RollerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

import java.util.function.Supplier;

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
  // Controller
  private final Vision vision;
  private final Shooter shooter;
  private final GameState gamestate;
  private final Indexer indexer;
  private final Intake intake;
  private final Turret turret;
  private final Climb climb;
  private SwerveDriveSimulation simDrive = null;

  // Controller
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private double intakeRollerValue = 0; // TODO FIX THESE NUMBERS
  private double indexerRollerValue = 0;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    gamestate = new GameState();
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
        intake =
            new Intake(
                new RollerIOTalonFX(
                    MotorIDConstants.INTAKE_ROLLER_LEFT_ID,
                    MotorIDConstants.INTAKE_ROLLER_RIGHT_ID),
                new PivotIOTalonFX(
                    MotorIDConstants.INTAKE_PIVOT_LEFT_ID,
                    MotorIDConstants.INTAKE_PIVOT_RIGHT_ID,
                    MotorIDConstants.INTAKE_ENCODER_ID));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision("placeholder", CameraConstants.placeHolderCamera));
        indexer = new Indexer(new IndexerIOSparkFlex(Constants.MotorIDConstants.INDEXER_MOTOR_ID));
        shooter =
            new Shooter(
                new ShooterIOTalonFX(
                    Constants.ShooterConstants.FLY_WHEEL_LEFT_ID,
                    Constants.ShooterConstants.FLY_WHEEL_RIGHT_ID,
                    Constants.ShooterConstants.HOOD_ID,
                    Constants.ShooterConstants.KICKER_ID));
        turret =
            new Turret(
                new TurretIOTalonFX(
                    MotorIDConstants.TURRET_MOTOR_ID,
                    TurretConstants.ENCODER_13_TOOTH,
                    TurretConstants.ENCODER_15_TOOTH));
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

        new ModuleIOTalonFXMapleSim(TunerConstants.BackRight, simDrive.getModules()[3]);
        indexer = new Indexer(new IndexerIO() {});
        shooter = new Shooter(new ShooterIOSim());
        intake = new Intake(new RollerIOSim(simDrive), new PivotIOSim());
        turret =
            new Turret(
                new TurretIOTalonFX(
                    MotorIDConstants.TURRET_MOTOR_ID,
                    TurretConstants.ENCODER_13_TOOTH,
                    TurretConstants.ENCODER_15_TOOTH));
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
        intake = new Intake(new RollerIO() {}, new PivotIO() {});
        indexer = new Indexer(new IndexerIO() {});
        shooter =
            new Shooter(
                new ShooterIOTalonFX(
                    Constants.ShooterConstants.FLY_WHEEL_LEFT_ID,
                    Constants.ShooterConstants.FLY_WHEEL_RIGHT_ID,
                    Constants.ShooterConstants.HOOD_ID,
                    Constants.ShooterConstants.KICKER_ID));        
        turret =
            new Turret(
                new TurretIOTalonFX(
                    MotorIDConstants.TURRET_MOTOR_ID,
                    TurretConstants.ENCODER_13_TOOTH,
                    TurretConstants.ENCODER_15_TOOTH));

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

    driverController
        .leftBumper()
        .whileTrue(intake.runIntakeAtSpeed(intakeRollerValue, PivotPositions.DEPLOYED));

    // Lock to 0 degrees when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    // lifts climb first time y pressed, lifts robot second time y pressed
    driverController
        .y()
        .onTrue(
            Commands.either(
                climb.setClimbPresetPosition(ClimbPositions.ReadyToClimbL1),
                climb.setClimbPresetPosition(ClimbPositions.FullyClimbedL1),
                () -> climb.getClimbPosition() != ClimbPositions.ReadyToClimbL1));
    // runs intake normaly
    driverController
        .leftTrigger()
        .whileTrue(
            intake.runIntakeAtSpeed(0.0, PivotIO.PivotPositions.DEPLOYED));
    // runs intake in reverse
    driverController
        .rightTrigger()
        .whileTrue(
            intake.runIntakeAtSpeed(-0.0,PivotIO.PivotPositions.DEPLOYED));
    //driverController.start().whileTrue(); add when there is a command for moving the pivot
    //set Turret aim to normal when y is pressed
    operatorController.y().onTrue(new TurretAngleAim(() -> drive.getPose(), turret));
    // sets gamedata manualy when stick pressed, left = red, right = blue
    operatorController
        .leftStick()
        .onTrue(
            gamestate.manualGameData(0));
    operatorController
        .rightStick()
        .onTrue(
            gamestate.manualGameData(1));
    //when right trigger pressed, run shooter and indexer to fire
    operatorController
        .rightTrigger()
        .whileTrue(
            shooter.runShooterAtSpeed(0.0)
            .alongWith(indexer.runIndexerWithAutoUnjam(0)));
    // uses d-pad to manualy angle turret
    operatorController.povRight().whileTrue(turret.setTurretAngleFastestPathCommand(0));
    operatorController.povDown().whileTrue(turret.setTurretAngleFastestPathCommand(270));
    operatorController.povLeft().whileTrue(turret.setTurretAngleFastestPathCommand(180));
    operatorController.povUp().whileTrue(turret.setTurretAngleFastestPathCommand(90));
    //while b is pressed set hood to position 0
    operatorController
        .b()
        .whileTrue(
            shooter.setHoodPosition(0)); //10 degrees
    //while a is pressed set hood to position 0
    operatorController
        .a()
        .whileTrue(
            shooter.setHoodPosition(0)); //10 degrees
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

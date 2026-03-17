package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootAtAngleSim;
import frc.robot.commands.ShooterAutoAimSequence;
import frc.robot.commands.TurretAngleAim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ZoneLogic;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXMapleSim;
import frc.robot.subsystems.gamestate.GameState;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.PivotIO;
import frc.robot.subsystems.intake.PivotIOSim;
import frc.robot.subsystems.intake.PivotIOTalonFX;
import frc.robot.subsystems.intake.RollerIO;
import frc.robot.subsystems.intake.RollerIOSim;
import frc.robot.subsystems.intake.RollerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.ObjectDetection;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private final GameState gamestate;
  private final Indexer indexer;
  private final Intake intake;
  private ObjectDetection objectDetection;
  private final Turret turret;
  private final Hood hood;
  private SwerveDriveSimulation simDrive;
  private IntakeSimulation simIntake;
  private ShootAtAngleSim simShooter;
  private final InterpTables interpTables;
  private final ZoneLogic zones;

  private final AutoFactory autoFactory;
  // Controller
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Dashboard inputs

  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    gamestate = new GameState();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
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
                new VisionIOPhotonVision("Port", VisionConstants.PORT_CAMERA_POSE),
                new VisionIOPhotonVision("Starboard", VisionConstants.STARBOARD_CAMERA_POSE));
        indexer =
            new Indexer(
                new IndexerIOTalonFX(
                    MotorIDConstants.INDEXER1_MOTOR_ID, MotorIDConstants.INDEXER2_MOTOR_ID));
        intake =
            new Intake(
                new RollerIOTalonFX(
                    MotorIDConstants.INTAKE_ROLLER_LEFT_ID,
                    MotorIDConstants.INTAKE_ROLLER_RIGHT_ID),
                new PivotIOTalonFX(
                    MotorIDConstants.INTAKE_PIVOT_LEFT_ID, MotorIDConstants.INTAKE_ENCODER_ID));
        shooter =
            new Shooter(
                new ShooterIOTalonFX(
                    Constants.ShooterConstants.FLY_WHEEL_LEFT_ID,
                    Constants.ShooterConstants.FLY_WHEEL_RIGHT_ID,
                    Constants.ShooterConstants.KICKER_ID));
        turret =
            new Turret(
                new TurretIOTalonFX(
                    MotorIDConstants.TURRET_MOTOR_ID,
                    TurretConstants.ENCODER_13_TOOTH,
                    TurretConstants.ENCODER_15_TOOTH));
        hood =
            new Hood(
                new HoodIOTalonFX(
                    Constants.ShooterConstants.HOOD_ID,
                    Constants.ShooterConstants.HOOD_ENCODER_ID));
        interpTables = new InterpTables();
        zones = new ZoneLogic(drive);

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations1
        simDrive =
            new SwerveDriveSimulation(
                Drive.mapleSimConfig, new Pose2d(new Translation2d(3, 3), new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(simDrive);
        simIntake =
            IntakeSimulation.OverTheBumperIntake(
                // Specify the type of game pieces that the intake can collect
                SimConstants.GAMEPIECE,
                // Specify the drivetrain to which this intake is attached
                simDrive,
                // Width of the intake
                Meters.of(SimConstants.INTAKE_WIDTH),
                // The extension length of the intake beyond the robot's frame (when activated)
                Meters.of(SimConstants.INTAKE_LENGTH),
                // The intake is mounted on the back side of the chassis
                IntakeSimulation.IntakeSide.BACK, // flipped from FRONT
                // The intake can hold up to 50 Fuel
                SimConstants.HOPPER_CAPACITY);
        drive =
            new Drive(
                new GyroIOSim(simDrive.getGyroSimulation()),
                new ModuleIOTalonFXMapleSim(TunerConstants.FrontLeft, simDrive.getModules()[0]),
                new ModuleIOTalonFXMapleSim(TunerConstants.FrontRight, simDrive.getModules()[1]),
                new ModuleIOTalonFXMapleSim(TunerConstants.BackLeft, simDrive.getModules()[2]),
                new ModuleIOTalonFXMapleSim(TunerConstants.BackRight, simDrive.getModules()[3]));
        intake = new Intake(new RollerIOSim(simIntake), new PivotIOSim());
        indexer = new Indexer(new IndexerIO() {});
        hood = new Hood(new HoodIOSim());
        shooter = new Shooter(new ShooterIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOSim(
                    "Camera",
                    VisionConstants.PORT_CAMERA_POSE,
                    simDrive::getSimulatedDriveTrainPose));
        turret = new Turret(new TurretIOSim());
        simShooter =
            new ShootAtAngleSim(simIntake, simDrive, turret, 6000, Units.degreesToRadians(45));
        interpTables = new InterpTables();
        zones = new ZoneLogic(drive);

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
                new VisionIOPhotonVision("placeholder", VisionConstants.PORT_CAMERA_POSE));
        intake = new Intake(new RollerIO() {}, new PivotIO() {});
        indexer = new Indexer(new IndexerIO() {});
        shooter =
            new Shooter(
                new ShooterIOTalonFX(
                    Constants.ShooterConstants.FLY_WHEEL_LEFT_ID,
                    Constants.ShooterConstants.FLY_WHEEL_RIGHT_ID,
                    Constants.ShooterConstants.KICKER_ID));
        turret =
            new Turret(
                new TurretIOTalonFX(
                    MotorIDConstants.TURRET_MOTOR_ID,
                    TurretConstants.ENCODER_13_TOOTH,
                    TurretConstants.ENCODER_15_TOOTH));
        hood = new Hood(new HoodIO() {});
        interpTables = new InterpTables();
        zones = new ZoneLogic(drive);

        break;

      default:
        throw new IllegalStateException("Unexpected value: " + Constants.currentMode);
    }
    // Set up auto routines
    NamedCommands.registerCommand("intake", intake.runRollerAtVoltage(6.0));
    NamedCommands.registerCommand(
        "shooter",
        new ShooterAutoAimSequence(
            shooter,
            hood,
            interpTables.shooterSpeedInterp,
            interpTables.hoodAngleInterp,
            () -> drive.getPose(),
            FieldConstants.HUB_POSE_BLUE,
            drive));
    NamedCommands.registerCommand(
        "turret",
        new TurretAngleAim(
                () -> drive.getPose(),
                turret,
                () -> FieldConstants.HUB_POSE_BLUE,
                drive,
                interpTables.timeOfFlightInterp)
            .withTimeout(2)
            .andThen(Commands.runOnce(() -> indexer.setVoltage(5))));
    NamedCommands.registerCommand("indexer", Commands.runOnce(() -> indexer.setVoltage(5)));
    NamedCommands.registerCommand(
        "deploy", intake.setPivotPosition(PivotIO.PivotPositions.DEPLOYED).withTimeout(1.5));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
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
    autoChooser.addOption("Turret SysId QF", turret.quasistatic(Direction.kForward));
    autoChooser.addOption("Turret SysId QR", turret.quasistatic(Direction.kReverse));
    autoChooser.addOption("Turret SysId DF", turret.dynamic(Direction.kForward));
    autoChooser.addOption("Turret SysId DR", turret.dynamic(Direction.kReverse));

    autoFactory =
        new AutoFactory(
            () -> drive.getPose(), (pose) -> drive.setPose(pose), drive::choreoDrive, true, drive);

    Autos autoRoutines =
        new Autos(
            autoFactory,
            drive,
            intake,
            shooter,
            indexer,
            hood,
            vision,
            turret,
            interpTables.shooterSpeedInterp,
            interpTables.hoodAngleInterp);

    autoRoutines.buildAutoChooser();
    autoRoutines.populateChooser(autoChooser);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}. TODO set values for motors
   */
  private void configureButtonBindings() {
    driverController
        .x()
        .whileTrue(
            Commands.run(
                    () -> {
                      if (zones.isInTrench(() -> drive.getPose())) {
                        hood.setHoodPos(0.0);
                      }
                    })
                .alongWith(
                    new TurretAngleAim(
                        () -> drive.getPose(),
                        turret,
                        () -> zones.getZoneTargetPose(),
                        drive,
                        interpTables.timeOfFlightInterp)));
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    driverController
        .leftTrigger()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                0.3));
    intake.setDefaultCommand(intake.setPivotPosition(PivotIO.PivotPositions.DEPLOYED));
    driverController.rightTrigger().whileTrue(intake.runRollerAtVoltage(6.0));
    operatorController
        .leftTrigger()
        .whileTrue(
            new TurretAngleAim(
                () -> drive.getPose(),
                turret,
                () -> FieldConstants.HUB_POSE_BLUE,
                drive,
                interpTables.timeOfFlightInterp));
    operatorController
        .rightTrigger()
        .whileTrue(
            new ShooterAutoAimSequence(
                shooter,
                hood,
                interpTables.shooterSpeedInterp,
                interpTables.hoodAngleInterp,
                () -> drive.getPose(),
                FieldConstants.HUB_POSE_BLUE,
                drive));
    operatorController.rightBumper().whileTrue(intake.runRollerAtVoltage(-6));
    operatorController.b().whileTrue(indexer.runIndexer(5));
    operatorController.x().whileTrue(intake.setPivotPosition(PivotIO.PivotPositions.DONTBREAK));
    operatorController.leftBumper().whileTrue(shooter.runShooterAtSpeed(35));
    operatorController.povUp().onTrue(turret.setTurretAngleFastestPathCommand(0));
    operatorController.povDown().onTrue(turret.setTurretAngleFastestPathCommand(180));
    operatorController.a().onTrue(hood.setHoodPosition(0.0));
    driverController
        .povUp()
        .onTrue(hood.incrementhoodCommand(1).onlyIf(() -> DriverStation.isTest()));
    driverController
        .povDown()
        .onTrue(hood.incrementhoodCommand(-1).onlyIf(() -> DriverStation.isTest()));
    driverController
        .povLeft()
        .onTrue(shooter.incrementShooterCommand(1).onlyIf(() -> DriverStation.isTest()));
    driverController
        .povRight()
        .onTrue(shooter.incrementShooterCommand(-1).onlyIf(() -> DriverStation.isTest()));
    driverController
        .rightBumper()
        .whileTrue(shooter.setKickerSpeedCommand(1).onlyIf(() -> DriverStation.isTest()));
    driverController
        .leftBumper()
        .whileTrue(indexer.runIndexer(5).onlyIf(() -> DriverStation.isTest()));
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

  AutoRoutine testAuto() {
    AutoRoutine routine = autoFactory.newRoutine("TEST_AUTO2");
    AutoTrajectory testPath = routine.trajectory("TEST");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                testPath.resetOdometry(),
                testPath.cmd(),
                Commands.runOnce(() -> drive.stop()),
                new ShooterAutoAimSequence(
                    shooter,
                    hood,
                    interpTables.shooterSpeedInterp,
                    interpTables.hoodAngleInterp,
                    () -> drive.getPose(),
                    FieldConstants.HUB_POSE_BLUE,
                    drive)));
    return routine;
  }
}

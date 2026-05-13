package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.qelib.SpatialAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SimConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXMapleSim;
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
  private SwerveDriveSimulation simDrive;
  private IntakeSimulation simIntake;

  // private final AutoFactory autoFactory;
  // private final SpatialAutoBuilder spatialAutoBuilder;
  // private final Map<String, Command> eventMap;
  // Controller
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Dashboard inputs

  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVision("placeholder", VisionConstants.PORT_CAMERA_POSE));

        break;

      default:
        throw new IllegalStateException("Unexpected value: " + Constants.currentMode);
    }
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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

    // autoFactory =
    //     new AutoFactory(
    //         () -> drive.getPose(), (pose) -> drive.setPose(pose), drive::choreoDrive, true,
    // drive);
    // CommandScheduler.getInstance().schedule(autoFactory.warmupCmd());

    // spatialAutoBuilder = new SpatialAutoBuilder();
    // eventMap = new HashMap<String, Command>();
    // eventMap.put("intake", intake.runIntakeAtSpeed(100, PivotPositions.DEPLOYED));
    // spatialAutoBuilder.configure(
    //     () -> drive.getPose(), (speeds) -> drive.runVelocity(speeds), eventMap, 5, 5, 5);
    // autoChooser.addOption("spatialTEst", spatialAutoBuilder.buildPath("TEST"));
    // Autos autoRoutines =
    //     new Autos(
    //         autoFactory,
    //         drive,
    //         intake,
    //         shooter,
    //         indexer,
    //         hood,
    //         vision,
    //         turret,
    //         interpTables.shooterSpeedHubInterp,
    //         interpTables.hoodAngleHubInterp,
    //         interpTables.timeOfFlightInterp);

    // autoRoutines.buildAutoChooser();
    // autoRoutines.populateChooser(autoChooser);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}. TODO set values for motors
   */
  private void configureButtonBindings() {

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            0.1));
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
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

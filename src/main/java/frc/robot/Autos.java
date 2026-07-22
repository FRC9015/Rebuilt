package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.ShootAtAngleSim;
import frc.robot.commands.ShooterAutoAimSequence;
import frc.robot.commands.TurretAngleAim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.PivotIO;
import frc.robot.subsystems.intake.PivotIO.PivotPositions;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private static final double AUTO_SHOOT_PERIOD_SECONDS = 0.1;
  private static final double AUTO_SHOOT_WINDOW_SECONDS = 2.5;

  // public enum TrajChoices {
  //   TEST,
  //   CENTER_RUSH_LEFT,
  //   CENTER_RUSH_RIGHT,
  //   DEPOT_LEFT,
  //   DEPOT_CENTER,
  //   HP_RIGHT;

  //   private static final Map<TrajChoices, Trajectory<?>> trajMap;

  //   static {
  //     trajMap = new HashMap<TrajChoices, Trajectory<?>>();
  //     for (TrajChoices auto : EnumSet.allOf(TrajChoices.class)) {
  //       trajMap.put(auto, Choreo.loadTrajectory(auto.name()).get());
  //     }
  //   }
  // }

  // public class AutonomousRoutines {
  private final AutoFactory autoFactory;
  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Hood hood;
  private final Vision vision;
  private final Turret turret;
  private final ShootAtAngleSim simShooter;
  private final Supplier<Pose2d> autoAimPoseSupplier;
  private final InterpolatingTreeMap<Double, Double> shooterInterp;
  private final InterpolatingTreeMap<Double, Double> hoodInterp;
  private final InterpolatingTreeMap<Double, Double> timeOfFlightInterp;

  /**
   * Constructor for AutonomousRoutines.
   *
   * @param autoFactory The factory for creating auto routines.
   * @param drive The drive subsystem.
   * @param intake The intake subsystem.
   * @param shooter The shooter subsystem.
   * @param indexer The indexer subsystem.
   * @param hood The hood subsystem.
   * @param vision The vision subsystem.
   * @param turret The turret subsystem.
   * @param shooterInterp The shooter interpolation table.
   * @param hoodInterp The hood interpolation table.
   */
  Autos(
      AutoFactory autoFactory,
      Drive drive,
      Intake intake,
      Shooter shooter,
      Indexer indexer,
      Hood hood,
      Vision vision,
      Turret turret,
      ShootAtAngleSim simShooter,
      Supplier<Pose2d> autoAimPoseSupplier,
      InterpolatingTreeMap<Double, Double> shooterInterp,
      InterpolatingTreeMap<Double, Double> hoodInterp,
      InterpolatingTreeMap<Double, Double> timeOfFlightInterp) {
    this.autoFactory = autoFactory;
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.indexer = indexer;
    this.hood = hood;
    this.vision = vision;
    this.turret = turret;
    this.simShooter = simShooter;
    this.autoAimPoseSupplier = autoAimPoseSupplier;
    this.shooterInterp = shooterInterp;
    this.hoodInterp = hoodInterp;
    this.timeOfFlightInterp = timeOfFlightInterp;
  }

  /**
   * Auto for testing purposes
   *
   * @return Routine Command
   * @see AutoRoutine
   */
  public Command testAuto() {
    AutoRoutine routine = autoFactory.newRoutine("TEST_AUTO");
    AutoTrajectory testPath = routine.trajectory(Choreo.loadTrajectory("TEST").get());

    routine
        .active()
        .onTrue(
            Commands.sequence(
                testPath.resetOdometry(),
                Commands.deadline(testPath.cmd(), getAutoIntakeCommand()),
                Commands.runOnce(() -> drive.stop()),
                Commands.deadline(getAutoShootCommand(), getAutoAimCommand())));
    return routine.cmd();
  }

  /**
   * Center Rush Left Auto
   *
   * @return Routine Command
   * @see AutoRoutine
   */
  public Command centerrushLeft() {
    AutoRoutine routine = autoFactory.newRoutine("CENTER_RUSH_LEFT");
    AutoTrajectory centerRush = routine.trajectory(Choreo.loadTrajectory("CENTERRUSH_LEFT").get());
    centerRush.atTime("Marker").onTrue(intake.runIntakeAtSpeed(100, PivotPositions.DEPLOYED));
    routine
        .active()
        .onTrue(
            Commands.sequence(
                centerRush.resetOdometry(),
                Commands.deadline(centerRush.cmd(), getAutoIntakeCommand()),
                Commands.runOnce(() -> drive.stop()),
                new TurretAngleAim(
                        autoAimPoseSupplier,
                        turret,
                        () -> FieldConstants.HUB_POSE_BLUE,
                        drive,
                        timeOfFlightInterp)
                    .withTimeout(1.5),
                Commands.deadline(getAutoShootCommand(), getAutoAimCommand())));

    return routine.cmd();
  }

  /**
   * Center Rush Right Auto
   *
   * @return Routine Command
   * @see AutoRoutine
   */
  public Command centerrushRight() {
    AutoRoutine routine = autoFactory.newRoutine("CENTER_RUSH_RIGHT");
    AutoTrajectory centerRush = routine.trajectory(Choreo.loadTrajectory("CENTERRUSH_RIGHT").get());

    routine
        .active()
        .onTrue(
            Commands.sequence(
                centerRush.resetOdometry(),
                Commands.deadline(centerRush.cmd(), getAutoIntakeCommand()),
                Commands.runOnce(() -> drive.stop()),
                Commands.deadline(getAutoShootCommand(), getAutoAimCommand())));
    return routine.cmd();
  }

  /**
   * Depot Left Auto
   *
   * @return Routine Command
   * @see AutoRoutine
   */
  public Command depotLeft() {
    AutoRoutine routine = autoFactory.newRoutine("DEPOT_LEFT");
    AutoTrajectory depotTraj = routine.trajectory(Choreo.loadTrajectory("DEPOT_LEFT").get());

    routine
        .active()
        .onTrue(
            Commands.sequence(
                depotTraj.resetOdometry(),
                Commands.deadline(depotTraj.cmd(), getAutoIntakeCommand()),
                Commands.runOnce(() -> drive.stop()),
                Commands.deadline(getAutoShootCommand(), getAutoAimCommand())));
    return routine.cmd();
  }

  /**
   * Depot Center Auto
   *
   * @return Routine Command
   * @see AutoRoutine
   */
  public Command depotCenter() {
    AutoRoutine routine = autoFactory.newRoutine("DEPOT_CENTER");
    AutoTrajectory depot = routine.trajectory(Choreo.loadTrajectory("DEPOT_CENTER").get());
    routine
        .active()
        .onTrue(
            Commands.sequence(
                depot.resetOdometry(),
                Commands.deadline(depot.cmd(), getAutoIntakeCommand()),
                Commands.runOnce(() -> drive.stop()),
                Commands.deadline(getAutoShootCommand(), getAutoAimCommand())));
    return routine.cmd();
  }

  /**
   * HP Right (Outpost) Auto
   *
   * @return Routine Command
   * @see AutoRoutine
   */
  public Command hpRight() {
    AutoRoutine routine = autoFactory.newRoutine("HP_RIGHT");
    AutoTrajectory hp = routine.trajectory(Choreo.loadTrajectory("HP_RIGHT").get());
    routine
        .active()
        .onTrue(
            Commands.sequence(
                hp.resetOdometry(),
                Commands.deadline(hp.cmd(), getAutoIntakeCommand()),
                Commands.runOnce(() -> drive.stop()),
                Commands.deadline(getAutoShootCommand(), getAutoAimCommand())));
    return routine.cmd();
  }

  /**
   * HP Right (Outpost) Auto
   *
   * @return Routine Command
   * @see AutoRoutine
   */
  public Command centerLeftDepo() {
    AutoRoutine routine = autoFactory.newRoutine("CenterLeftDepo");
    AutoTrajectory cl = routine.trajectory(Choreo.loadTrajectory("CENTERRUSH_LEFT").get());
    AutoTrajectory de = routine.trajectory(Choreo.loadTrajectory("DEPOT_LEFT").get());
    cl.atTime("Marker").onTrue(intake.runIntakeAtSpeed(100, PivotPositions.DEPLOYED));
    de.atTime("intaking").onTrue(intake.runIntakeAtSpeed(100, PivotPositions.DEPLOYED));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                cl.resetOdometry(),
                Commands.deadline(cl.cmd(), getAutoIntakeCommand()),
                Commands.runOnce(() -> drive.stop()),
                Commands.parallel(
                        Commands.deadline(getAutoShootCommand(), getAutoAimCommand()),
                        intake.agitateIntakeCommand())
                    .withTimeout(4.0),
                Commands.deadline(de.cmd(), getAutoIntakeCommand()),
                Commands.runOnce(() -> drive.stop()),
                Commands.parallel(
                    Commands.deadline(getAutoShootCommand(), getAutoAimCommand()),
                    intake.agitateIntakeCommand())));
    return routine.cmd();
  }

  /**
   * Populates a given dashboard chooser with the choreo autos.
   *
   * @param chooser The dashboard chooser to populate.
   * @see LoggedDashboardChooser
   */
  public void populateChooser(LoggedDashboardChooser<Command> chooser) {
    // Get all methods in this class
    for (Method method : this.getClass().getDeclaredMethods()) {

      // Filter: Must return Command, must have 0 parameters, and must be public
      if (method.getReturnType().equals(Command.class)
          && method.getParameterCount() == 0
          && Modifier.isPublic(method.getModifiers())) {

        // Optional: Skip the populateChooser method itself if it matched
        if (method.getName().equals("populateChooser")
            || method.getName().equals("buildAutoChooser")) {
          continue;
        }

        String name = method.getName();

        // Add to the chooser using deferredProxy
        // This ensures the method is called ONLY when the auto starts
        chooser.addOption(
            name,
            Commands.deferredProxy(
                () -> {
                  try {
                    return (Command) method.invoke(this);
                  } catch (Exception e) {
                    System.err.println("Could not invoke auto method: " + name);
                    return Commands.none();
                  }
                }));
      }
    }
  }

  /**
   * Builds the AutoFactory using needed named commands. Should be ran before populating.
   *
   * @see AutoFactory
   */
  public void buildAutoChooser() {
    autoFactory.bind("Intake", getAutoIntakeCommand());
    autoFactory.bind("ShooterSpeed", getAutoAimCommand());
    autoFactory.bind("ShootBall", getAutoShootCommand());
    autoFactory.bind("deploy", intake.setPivotPosition(PivotIO.PivotPositions.DEPLOYED));
  }

  private Command getAutoIntakeCommand() {
    return intake.runIntakeAtSpeed(100, PivotPositions.DEPLOYED);
  }

  private Command getAutoShootCommand() {
    if (Constants.currentMode == Constants.Mode.SIM && simShooter != null) {
      return Commands.waitUntil(shooter::returnShooterAtSetpoint)
          .andThen(
              Commands.sequence(
                      Commands.runOnce(() -> simShooter.shootBalls()),
                      Commands.waitSeconds(AUTO_SHOOT_PERIOD_SECONDS))
                  .repeatedly())
          .withTimeout(AUTO_SHOOT_WINDOW_SECONDS);
    }

    return Commands.waitUntil(shooter::returnShooterAtSetpoint)
        .andThen(
            Commands.startEnd(() -> shooter.setKickerSpeed(6.0), () -> shooter.stopKicker())
                .alongWith(indexer.runIndexer(6.0)))
        .withTimeout(AUTO_SHOOT_WINDOW_SECONDS);
  }

  private Command getAutoAimCommand() {
    return new ShooterAutoAimSequence(
            shooter,
            hood,
            shooterInterp,
            hoodInterp,
            timeOfFlightInterp,
            autoAimPoseSupplier,
            () -> FieldConstants.HUB_POSE_BLUE,
            drive)
        .alongWith(
            new TurretAngleAim(
                autoAimPoseSupplier,
                turret,
                () -> FieldConstants.HUB_POSE_BLUE,
                drive,
                timeOfFlightInterp));
  }
}

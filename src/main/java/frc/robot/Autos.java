// package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.ShooterAutoAimSequence;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.PivotIO.PivotPositions;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.PhoenixUtil;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
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
  private final InterpolatingTreeMap<Double, Double> shooterInterp;
  private final InterpolatingTreeMap<Double, Double> hoodInterp;

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
      InterpolatingTreeMap<Double, Double> shooterInterp,
      InterpolatingTreeMap<Double, Double> hoodInterp) {
    this.autoFactory = autoFactory;
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.indexer = indexer;
    this.hood = hood;
    this.vision = vision;
    this.turret = turret;
    this.shooterInterp = shooterInterp;
    this.hoodInterp = hoodInterp;
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
                testPath.cmd(),
                Commands.runOnce(() -> drive.stop()),
                new ShooterAutoAimSequence(
                    shooter,
                    hood,
                    shooterInterp,
                    hoodInterp,
                    () -> drive.getPose(),
                    FieldConstants.HUB_POSE_BLUE,
                    drive)));
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
    AutoTrajectory centerRush = routine.trajectory(Choreo.loadTrajectory("CENTER_RUSH_LEFT").get());
    routine
        .active()
        .onTrue(
            Commands.sequence(
                centerRush.resetOdometry(),
                centerRush.cmd(),
                Commands.runOnce(() -> drive.stop()),
                new ShooterAutoAimSequence(
                    shooter,
                    hood,
                    shooterInterp,
                    hoodInterp,
                    () -> drive.getPose(),
                    FieldConstants.HUB_POSE_BLUE,
                    drive)));

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
    AutoTrajectory centerRush =
        routine.trajectory(Choreo.loadTrajectory("CENTER_RUSH_RIGHT").get());

    routine
        .active()
        .onTrue(
            Commands.sequence(
                centerRush.resetOdometry(),
                centerRush.cmd(),
                Commands.runOnce(() -> drive.stop()),
                new ShooterAutoAimSequence(
                    shooter,
                    hood,
                    shooterInterp,
                    hoodInterp,
                    () -> drive.getPose(),
                    FieldConstants.HUB_POSE_BLUE,
                    drive)));
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
                depotTraj.cmd(),
                Commands.runOnce(() -> drive.stop()),
                new ShooterAutoAimSequence(
                        shooter,
                        hood,
                        shooterInterp,
                        hoodInterp,
                        () -> drive.getPose(),
                        FieldConstants.HUB_POSE_BLUE,
                        drive)
                    .alongWith(indexer.runIndexer(6.0))));
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
                depot.cmd(),
                Commands.runOnce(() -> drive.stop()),
                new ShooterAutoAimSequence(
                    shooter,
                    hood,
                    shooterInterp,
                    hoodInterp,
                    () -> drive.getPose(),
                    FieldConstants.HUB_POSE_BLUE,
                    drive)));
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
                hp.cmd(),
                Commands.runOnce(() -> drive.stop()),
                new ShooterAutoAimSequence(
                    shooter,
                    hood,
                    shooterInterp,
                    hoodInterp,
                    () -> drive.getPose(),
                    FieldConstants.HUB_POSE_BLUE,
                    drive)));
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
    autoFactory.bind("Intake", intake.runIntakeAtSpeed(50, PivotPositions.DEPLOYED));
    autoFactory.bind(
        "ShooterSpeed",
        new ShooterAutoAimSequence(
            shooter,
            hood,
            shooterInterp,
            hoodInterp,
            () -> drive.getPose(),
            PhoenixUtil.isRed() ? FieldConstants.HUB_POSE_RED : FieldConstants.HUB_POSE_BLUE,
            drive));
    autoFactory.bind("ShootBall", indexer.runIndexer(6.0));
  }
}

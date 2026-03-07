package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
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

public class Autos {
  public enum AutoChoices {
    TEST_PATH(Choreo.loadTrajectory("TestPath").get()),
    CENTER_RUSH_LEFT(Choreo.loadTrajectory("CenterRush_Left").get()),
    CENTER_RUSH_RIGHT(Choreo.loadTrajectory("CenterRush_Right").get()),
    DEPOT_LEFT(Choreo.loadTrajectory("Depot_Left").get()),
    DEPOT_CENTER(Choreo.loadTrajectory("Depot_Center").get()),
    HP_CENTER(Choreo.loadTrajectory("HP_Right").get());

    private final Trajectory<?> traj;

    private AutoChoices(Trajectory<?> traj) {
      this.traj = traj;
    }

    public Trajectory<?> getTrajectory() {
      return traj;
    }
  }

  public class AutonomousRoutines {
    public static final AutoRoutine testAuto(
        Drive drive,
        Intake intake,
        Shooter shooter,
        Indexer indexer,
        Hood hood,
        Vision vision,
        Turret turret,
        InterpolatingTreeMap<Double, Double> shooterInterp,
        InterpolatingTreeMap<Double, Double> hoodInterp) {
      AutoRoutine routine = drive.autoFactory.newRoutine("TEST_AUTO");
      AutoTrajectory testPath = routine.trajectory(AutoChoices.TEST_PATH.getTrajectory());

      routine
          .active()
          .onTrue(
              Commands.sequence(
                  testPath.resetOdometry(),
                  testPath.cmd(),
                  Commands.runOnce(() -> drive.stop()),
                  Commands.parallel(
                      new ShooterAutoAimSequence(
                          shooter,
                          hood,
                          shooterInterp,
                          hoodInterp,
                          () -> drive.getPose(),
                          FieldConstants.HUB_POSE_BLUE,
                          drive),
                      shooter.setKickerSpeedCommand(1))));
      return routine;
    }

    public static final AutoRoutine centerrushLeftBlue(
        Drive drive,
        Intake intake,
        Shooter shooter,
        Indexer indexer,
        Hood hood,
        Vision vision,
        Turret turret,
        InterpolatingTreeMap<Double, Double> shooterInterp,
        InterpolatingTreeMap<Double, Double> hoodInterp) {
      AutoRoutine routine = drive.autoFactory.newRoutine("CENTER_RUSH_LEFT");
      AutoTrajectory centerRush = routine.trajectory(AutoChoices.CENTER_RUSH_LEFT.getTrajectory());
      routine
          .active()
          .onTrue(
              Commands.sequence(
                  centerRush.resetOdometry(),
                  centerRush.cmd(),
                  Commands.runOnce(() -> drive.stop()),
                  Commands.parallel(
                      new ShooterAutoAimSequence(
                          shooter,
                          hood,
                          shooterInterp,
                          hoodInterp,
                          () -> drive.getPose(),
                          FieldConstants.HUB_POSE_BLUE,
                          drive),
                      shooter.setKickerSpeedCommand(1))));

      return routine;
    }

    public static final AutoRoutine centerrushRightBlue(
        Drive drive,
        Intake intake,
        Shooter shooter,
        Indexer indexer,
        Hood hood,
        Vision vision,
        Turret turret,
        InterpolatingTreeMap<Double, Double> shooterInterp,
        InterpolatingTreeMap<Double, Double> hoodInterp) {
      AutoRoutine routine = drive.autoFactory.newRoutine("CENTER_RUSH_RIGHT");
      AutoTrajectory centerRush = routine.trajectory(AutoChoices.CENTER_RUSH_RIGHT.getTrajectory());

      routine
          .active()
          .onTrue(
              Commands.sequence(
                  centerRush.resetOdometry(),
                  centerRush.cmd(),
                  Commands.runOnce(() -> drive.stop()),
                  Commands.parallel(
                      new ShooterAutoAimSequence(
                          shooter,
                          hood,
                          shooterInterp,
                          hoodInterp,
                          () -> drive.getPose(),
                          FieldConstants.HUB_POSE_BLUE,
                          drive),
                      shooter.setKickerSpeedCommand(1))));
      return routine;
    }

    public static final AutoRoutine depotLeftBlue(
        Drive drive,
        Intake intake,
        Shooter shooter,
        Indexer indexer,
        Hood hood,
        Vision vision,
        Turret turret,
        InterpolatingTreeMap<Double, Double> shooterInterp,
        InterpolatingTreeMap<Double, Double> hoodInterp) {
      AutoRoutine routine = drive.autoFactory.newRoutine("DEPOT_LEFT");
      AutoTrajectory depotTraj = routine.trajectory(AutoChoices.DEPOT_LEFT.getTrajectory());

      routine
          .active()
          .onTrue(
              Commands.sequence(
                  depotTraj.resetOdometry(),
                  depotTraj.cmd(),
                  Commands.runOnce(() -> drive.stop()),
                  Commands.parallel(
                      new ShooterAutoAimSequence(
                          shooter,
                          hood,
                          shooterInterp,
                          hoodInterp,
                          () -> drive.getPose(),
                          FieldConstants.HUB_POSE_BLUE,
                          drive),
                      shooter.setKickerSpeedCommand(1))));
      return routine;
    }

    public static final AutoRoutine depotCenterBlue(
        Drive drive,
        Intake intake,
        Shooter shooter,
        Indexer indexer,
        Hood hood,
        Vision vision,
        Turret turret,
        InterpolatingTreeMap<Double, Double> shooterInterp,
        InterpolatingTreeMap<Double, Double> hoodInterp) {
      AutoRoutine routine = drive.autoFactory.newRoutine("DEPOT_CENTER");
      AutoTrajectory depot = routine.trajectory(AutoChoices.DEPOT_CENTER.getTrajectory());
      routine
          .active()
          .onTrue(
              Commands.sequence(
                  depot.resetOdometry(),
                  depot.cmd(),
                  Commands.runOnce(() -> drive.stop()),
                  Commands.parallel(
                      new ShooterAutoAimSequence(
                          shooter,
                          hood,
                          shooterInterp,
                          hoodInterp,
                          () -> drive.getPose(),
                          FieldConstants.HUB_POSE_BLUE,
                          drive),
                      shooter.setKickerSpeedCommand(1))));
      return routine;
    }

    public static final AutoRoutine hpRightBlue(
        Drive drive,
        Intake intake,
        Shooter shooter,
        Indexer indexer,
        Hood hood,
        Vision vision,
        Turret turret,
        InterpolatingTreeMap<Double, Double> shooterInterp,
        InterpolatingTreeMap<Double, Double> hoodInterp) {
      AutoRoutine routine = drive.autoFactory.newRoutine("HP_RIGHT");
      AutoTrajectory hp = routine.trajectory("HP_RIGHT");
      routine
          .active()
          .onTrue(
              Commands.sequence(
                  hp.resetOdometry(),
                  hp.cmd(),
                  Commands.runOnce(() -> drive.stop()),
                  Commands.parallel(
                      new ShooterAutoAimSequence(
                          shooter,
                          hood,
                          shooterInterp,
                          hoodInterp,
                          () -> drive.getPose(),
                          FieldConstants.HUB_POSE_BLUE,
                          drive),
                      shooter.setKickerSpeedCommand(1))));
      return routine;
    }

    public static final AutoRoutine centerrushLeftRed(
        Drive drive,
        Intake intake,
        Shooter shooter,
        Indexer indexer,
        Hood hood,
        Vision vision,
        Turret turret,
        InterpolatingTreeMap<Double, Double> shooterInterp,
        InterpolatingTreeMap<Double, Double> hoodInterp) {
      AutoRoutine routine = drive.autoFactory.newRoutine("CENTER_RUSH_LEFT");
      AutoTrajectory centerRush = routine.trajectory(AutoChoices.CENTER_RUSH_LEFT.getTrajectory());
      routine
          .active()
          .onTrue(
              Commands.sequence(
                  centerRush.resetOdometry(),
                  centerRush.cmd(),
                  Commands.runOnce(() -> drive.stop()),
                  Commands.parallel(
                      new ShooterAutoAimSequence(
                          shooter,
                          hood,
                          shooterInterp,
                          hoodInterp,
                          () -> drive.getPose(),
                          FieldConstants.HUB_POSE_BLUE,
                          drive),
                      shooter.setKickerSpeedCommand(1))));

      return routine;
    }

    public static final AutoRoutine centerrushRightRed(
        Drive drive,
        Intake intake,
        Shooter shooter,
        Indexer indexer,
        Hood hood,
        Vision vision,
        Turret turret,
        InterpolatingTreeMap<Double, Double> shooterInterp,
        InterpolatingTreeMap<Double, Double> hoodInterp) {
      AutoRoutine routine = drive.autoFactory.newRoutine("CENTER_RUSH_RIGHT");
      AutoTrajectory centerRush = routine.trajectory(AutoChoices.CENTER_RUSH_RIGHT.getTrajectory());

      routine
          .active()
          .onTrue(
              Commands.sequence(
                  centerRush.resetOdometry(),
                  centerRush.cmd(),
                  Commands.runOnce(() -> drive.stop()),
                  Commands.parallel(
                      new ShooterAutoAimSequence(
                          shooter,
                          hood,
                          shooterInterp,
                          hoodInterp,
                          () -> drive.getPose(),
                          FieldConstants.HUB_POSE_RED,
                          drive),
                      shooter.setKickerSpeedCommand(1))));
      return routine;
    }

    public static final AutoRoutine depotLeftRed(
        Drive drive,
        Intake intake,
        Shooter shooter,
        Indexer indexer,
        Hood hood,
        Vision vision,
        Turret turret,
        InterpolatingTreeMap<Double, Double> shooterInterp,
        InterpolatingTreeMap<Double, Double> hoodInterp) {
      AutoRoutine routine = drive.autoFactory.newRoutine("DEPOT_LEFT");
      AutoTrajectory depotTraj = routine.trajectory(AutoChoices.DEPOT_LEFT.getTrajectory());

      routine
          .active()
          .onTrue(
              Commands.sequence(
                  depotTraj.resetOdometry(),
                  depotTraj.cmd(),
                  Commands.runOnce(() -> drive.stop()),
                  Commands.parallel(
                      new ShooterAutoAimSequence(
                          shooter,
                          hood,
                          shooterInterp,
                          hoodInterp,
                          () -> drive.getPose(),
                          FieldConstants.HUB_POSE_RED,
                          drive),
                      shooter.setKickerSpeedCommand(1))));
      return routine;
    }

    public static final AutoRoutine depotCenteRed(
        Drive drive,
        Intake intake,
        Shooter shooter,
        Indexer indexer,
        Hood hood,
        Vision vision,
        Turret turret,
        InterpolatingTreeMap<Double, Double> shooterInterp,
        InterpolatingTreeMap<Double, Double> hoodInterp) {
      AutoRoutine routine = drive.autoFactory.newRoutine("DEPOT_CENTER");
      AutoTrajectory depot = routine.trajectory(AutoChoices.DEPOT_CENTER.getTrajectory());
      routine
          .active()
          .onTrue(
              Commands.sequence(
                  depot.resetOdometry(),
                  depot.cmd(),
                  Commands.runOnce(() -> drive.stop()),
                  Commands.parallel(
                      new ShooterAutoAimSequence(
                          shooter,
                          hood,
                          shooterInterp,
                          hoodInterp,
                          () -> drive.getPose(),
                          FieldConstants.HUB_POSE_RED,
                          drive),
                      shooter.setKickerSpeedCommand(1))));
      return routine;
    }

    public static final AutoRoutine hpRightRed(
        Drive drive,
        Intake intake,
        Shooter shooter,
        Indexer indexer,
        Hood hood,
        Vision vision,
        Turret turret,
        InterpolatingTreeMap<Double, Double> shooterInterp,
        InterpolatingTreeMap<Double, Double> hoodInterp) {
      AutoRoutine routine = drive.autoFactory.newRoutine("HP_RIGHT");
      AutoTrajectory hp = routine.trajectory("HP_RIGHT");
      routine
          .active()
          .onTrue(
              Commands.sequence(
                  hp.resetOdometry(),
                  hp.cmd(),
                  Commands.runOnce(() -> drive.stop()),
                  Commands.parallel(
                      new ShooterAutoAimSequence(
                          shooter,
                          hood,
                          shooterInterp,
                          hoodInterp,
                          () -> drive.getPose(),
                          FieldConstants.HUB_POSE_RED,
                          drive),
                      shooter.setKickerSpeedCommand(1))));
      return routine;
    }
  }

  public static void populateChooser(
      AutoChooser chooser,
      Drive drive,
      Intake intake,
      Shooter shooter,
      Indexer indexer,
      Hood hood,
      Vision vision,
      Turret turret,
      InterpolatingTreeMap<Double, Double> shooterInterp,
      InterpolatingTreeMap<Double, Double> hoodInterp) {
    for (Method method : AutonomousRoutines.class.getDeclaredMethods()) {

      if (!Modifier.isStatic(method.getModifiers())) continue;

      String name = method.getName();

      chooser.addRoutine(
          name,
          () -> {
            try {
              return (AutoRoutine)
                  method.invoke(null, drive, intake, shooter, indexer, vision, turret);
            } catch (Exception e) {
              throw new RuntimeException(e);
            }
          });
    }
  }

  public static void buildAutoChooser(
      AutoChooser chooser,
      Drive drive,
      Intake intake,
      Shooter shooter,
      Indexer indexer,
      Hood hood,
      Vision vision,
      Turret turret,
      InterpolatingTreeMap<Double, Double> shooterInterp,
      InterpolatingTreeMap<Double, Double> hoodInterp) {
    drive.autoFactory.bind(
        "Intake",
        intake.runRollerAtVoltage(8).alongWith(intake.setPivotPosition(PivotPositions.DEPLOYED)));
    drive.autoFactory.bind(
        "ShooterSpeed",
        new ShooterAutoAimSequence(
            shooter,
            hood,
            shooterInterp,
            hoodInterp,
            () -> drive.getPose(),
            PhoenixUtil.isRed() ? FieldConstants.HUB_POSE_RED : FieldConstants.HUB_POSE_BLUE,
            drive));
    drive.autoFactory.bind(
        "ShootBall", shooter.setKickerSpeedCommand(1).alongWith(indexer.runIndexer(6.0)));
  }
}

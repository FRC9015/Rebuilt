package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
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
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

public class Autos {
  public enum AutoChoices {
    TEST,
    CENTER_RUSH_LEFT,
    CENTER_RUSH_RIGHT,
    DEPOT_LEFT,
    DEPOT_CENTER,
    HP_RIGHT;

    private static final Map<AutoChoices, Trajectory<?>> trajMap;

    static {
      trajMap = new HashMap<AutoChoices, Trajectory<?>>();
      for (AutoChoices auto : EnumSet.allOf(AutoChoices.class)) {
        trajMap.put(auto, Choreo.loadTrajectory(auto.name()).get());
      }
    }
  }

  public class AutonomousRoutines {
    public static final AutoRoutine TestAuto(
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
      AutoRoutine routine = autoFactory.newRoutine("TEST_AUTO");
      AutoTrajectory testPath = routine.trajectory(AutoChoices.trajMap.get(AutoChoices.TEST));

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
      return routine;
    }

    public static final AutoRoutine CenterRushLeft(
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
      AutoRoutine routine = autoFactory.newRoutine("CENTER_RUSH_LEFT");
      AutoTrajectory centerRush =
          routine.trajectory(AutoChoices.trajMap.get(AutoChoices.CENTER_RUSH_LEFT));
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

      return routine;
    }

    public static final AutoRoutine CenterRushRight(
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
      AutoRoutine routine = autoFactory.newRoutine("CENTER_RUSH_RIGHT");
      AutoTrajectory centerRush =
          routine.trajectory(AutoChoices.trajMap.get(AutoChoices.CENTER_RUSH_RIGHT));

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
      return routine;
    }

    public static final AutoRoutine DepotLeft(
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
      AutoRoutine routine = autoFactory.newRoutine("DEPOT_LEFT");
      AutoTrajectory depotTraj =
          routine.trajectory(AutoChoices.trajMap.get(AutoChoices.DEPOT_LEFT));

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
                      drive)));
      return routine;
    }

    public static final AutoRoutine DepotCenter(
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
      AutoRoutine routine = autoFactory.newRoutine("DEPOT_CENTER");
      AutoTrajectory depot = routine.trajectory(AutoChoices.trajMap.get(AutoChoices.DEPOT_CENTER));
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
      return routine;
    }

    public static final AutoRoutine HPRight(
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
      AutoRoutine routine = autoFactory.newRoutine("HP_RIGHT");
      AutoTrajectory hp = routine.trajectory(AutoChoices.trajMap.get(AutoChoices.HP_RIGHT));
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
      return routine;
    }

    //     public static final AutoRoutine centerrushLeftRed(
    //         AutoFactory autoFactory,
    //         Drive drive,
    //         Intake intake,
    //         Shooter shooter,
    //         Indexer indexer,
    //         Hood hood,
    //         Vision vision,
    //         Turret turret,
    //         InterpolatingTreeMap<Double, Double> shooterInterp,
    //         InterpolatingTreeMap<Double, Double> hoodInterp) {
    //       AutoRoutine routine = autoFactory.newRoutine("CENTER_RUSH_LEFT");
    //       //   AutoTrajectory centerRush =
    //       // routine.trajectory(AutoChoices.CENTER_RUSH_LEFT.getTrajectory());
    //       AutoTrajectory centerRush = routine.trajectory(CRL);

    //       routine
    //           .active()
    //           .onTrue(
    //               Commands.sequence(
    //                   centerRush.resetOdometry(),
    //                   centerRush.cmd(),
    //                   Commands.runOnce(() -> drive.stop()),
    //                   new ShooterAutoAimSequence(
    //                       shooter,
    //                       hood,
    //                       shooterInterp,
    //                       hoodInterp,
    //                       () -> drive.getPose(),
    //                       FieldConstants.HUB_POSE_BLUE,
    //                       drive)));

    //       return routine;
    //     }

    //     public static final AutoRoutine centerrushRightRed(
    //         AutoFactory autoFactory,
    //         Drive drive,
    //         Intake intake,
    //         Shooter shooter,
    //         Indexer indexer,
    //         Hood hood,
    //         Vision vision,
    //         Turret turret,
    //         InterpolatingTreeMap<Double, Double> shooterInterp,
    //         InterpolatingTreeMap<Double, Double> hoodInterp) {
    //       AutoRoutine routine = autoFactory.newRoutine("CENTER_RUSH_RIGHT");
    //       AutoTrajectory centerRush =
    // routine.trajectory(AutoChoices.CENTER_RUSH_RIGHT.getTrajectory());

    //       routine
    //           .active()
    //           .onTrue(
    //               Commands.sequence(
    //                   centerRush.resetOdometry(),
    //                   centerRush.cmd(),
    //                   Commands.runOnce(() -> drive.stop()),
    //                   new ShooterAutoAimSequence(
    //                       shooter,
    //                       hood,
    //                       shooterInterp,
    //                       hoodInterp,
    //                       () -> drive.getPose(),
    //                       FieldConstants.HUB_POSE_BLUE,
    //                       drive)));
    //       return routine;
    //     }

    //     public static final AutoRoutine depotLeftRed(
    //         AutoFactory autoFactory,
    //         Drive drive,
    //         Intake intake,
    //         Shooter shooter,
    //         Indexer indexer,
    //         Hood hood,
    //         Vision vision,
    //         Turret turret,
    //         InterpolatingTreeMap<Double, Double> shooterInterp,
    //         InterpolatingTreeMap<Double, Double> hoodInterp) {
    //       AutoRoutine routine = autoFactory.newRoutine("DEPOT_LEFT");
    //       AutoTrajectory depotTraj = routine.trajectory(AutoChoices.DEPOT_LEFT.getTrajectory());

    //       routine
    //           .active()
    //           .onTrue(
    //               Commands.sequence(
    //                   depotTraj.resetOdometry(),
    //                   depotTraj.cmd(),
    //                   Commands.runOnce(() -> drive.stop()),
    //                   new ShooterAutoAimSequence(
    //                       shooter,
    //                       hood,
    //                       shooterInterp,
    //                       hoodInterp,
    //                       () -> drive.getPose(),
    //                       FieldConstants.HUB_POSE_BLUE,
    //                       drive)));
    //       return routine;
    //     }

    //     public static final AutoRoutine depotCenteRed(
    //         AutoFactory autoFactory,
    //         Drive drive,
    //         Intake intake,
    //         Shooter shooter,
    //         Indexer indexer,
    //         Hood hood,
    //         Vision vision,
    //         Turret turret,
    //         InterpolatingTreeMap<Double, Double> shooterInterp,
    //         InterpolatingTreeMap<Double, Double> hoodInterp) {
    //       AutoRoutine routine = autoFactory.newRoutine("DEPOT_CENTER");
    //       AutoTrajectory depot = routine.trajectory(AutoChoices.DEPOT_CENTER.getTrajectory());
    //       routine
    //           .active()
    //           .onTrue(
    //               Commands.sequence(
    //                   depot.resetOdometry(),
    //                   depot.cmd(),
    //                   Commands.runOnce(() -> drive.stop()),
    //                   new ShooterAutoAimSequence(
    //                       shooter,
    //                       hood,
    //                       shooterInterp,
    //                       hoodInterp,
    //                       () -> drive.getPose(),
    //                       FieldConstants.HUB_POSE_BLUE,
    //                       drive)));
    //       return routine;
    //     }

    //     public static final AutoRoutine hpRightRed(
    //         AutoFactory autoFactory,
    //         Drive drive,
    //         Intake intake,
    //         Shooter shooter,
    //         Indexer indexer,
    //         Hood hood,
    //         Vision vision,
    //         Turret turret,
    //         InterpolatingTreeMap<Double, Double> shooterInterp,
    //         InterpolatingTreeMap<Double, Double> hoodInterp) {
    //       AutoRoutine routine = autoFactory.newRoutine("HP_RIGHT");
    //       AutoTrajectory hp = routine.trajectory("HP_RIGHT");
    //       routine
    //           .active()
    //           .onTrue(
    //               Commands.sequence(
    //                   hp.resetOdometry(),
    //                   hp.cmd(),
    //                   Commands.runOnce(() -> drive.stop()),
    //                   new ShooterAutoAimSequence(
    //                       shooter,
    //                       hood,
    //                       shooterInterp,
    //                       hoodInterp,
    //                       () -> drive.getPose(),
    //                       FieldConstants.HUB_POSE_BLUE,
    //                       drive)));
    //       return routine;
    //     }
  }

  public static void populateChooser(
      AutoFactory autoFactory,
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

      if (!Modifier.isStatic(method.getModifiers()) || method.isSynthetic()) continue;
      System.out.println(method.getName());
      String name = method.getName();

      chooser.addRoutine(
          name,
          () -> {
            try {
              return (AutoRoutine)
                  method.invoke(
                      null,
                      autoFactory,
                      drive,
                      intake,
                      shooter,
                      indexer,
                      vision,
                      turret,
                      shooterInterp,
                      hoodInterp);
            } catch (Exception e) {
              throw new RuntimeException(e);
            }
          });
    }
  }

  public static void buildAutoChooser(
      AutoFactory autoFactory,
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
    autoFactory.bind(
        "Intake",
        intake.runRollerAtVoltage(8).alongWith(intake.setPivotPosition(PivotPositions.DEPLOYED)));
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
    autoFactory.bind(
        "ShootBall", shooter.setKickerSpeedCommand(1).alongWith(indexer.runIndexer(6.0)));
  }
}

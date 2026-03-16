package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.Trajectory;
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
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

  public Command testAuto() {
    AutoRoutine routine = autoFactory.newRoutine("TEST_AUTO");
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
                    shooterInterp,
                    hoodInterp,
                    () -> drive.getPose(),
                    FieldConstants.HUB_POSE_BLUE,
                    drive)));
    return routine.cmd();
  }

  public Command CenterRushLeft() {
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

    return routine.cmd();
  }

  public Command CenterRushRight() {
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
    return routine.cmd();
  }

  public Command DepotLeft() {
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
    return routine.cmd();
  }

  public Command DepotCenter() {
    AutoRoutine routine = autoFactory.newRoutine("DEPOT_CENTER");
    AutoTrajectory depot =
  routine.trajectory(AutoChoices.trajMap.get(AutoChoices.DEPOT_CENTER));
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

  public Command HPRight() {
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
    return routine.cmd();
  }

  public static void populateChooser(LoggedDashboardChooser<Command> chooser) {
    for (Method method : Autos.class.getDeclaredMethods()) {

      if (method.isSynthetic()) continue;
      System.out.println(method.getName());
      String name = method.getName();
      Method methodCall = () -> {
        try {
          return (Command)method.invoke(null);
        } catch (Exception e){
          return e;
        }
      };
      chooser.addOption(
          name, Commands.deferredProxy(() -> (Command)method.invoke(null)));
    }
  }

  public void buildAutoChooser() {
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

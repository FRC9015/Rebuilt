package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Zones;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ZoneCommands extends Command {

  private final Drive drive;
  private final Supplier<Pose2d> pose;
  private final Hood hood;
  private boolean hubLock = false;
  //   private final Supplier<Boolean> run;
  //   private final Supplier<Boolean> override;
  //   private final Turret turret;
  //   private final DriverStation.Alliance alliance;

  public ZoneCommands(Supplier<Pose2d> pose, Drive drive, Hood hood
      //   Supplier<Boolean> run,
      //   Supplier<Boolean> override,
      //   Turret turret
      ) {

    this.pose = pose;
    this.drive = drive;
    this.hood = hood;
    // this.run = run;
    // this.override = override;
    // this.turret = turret;
    // this.alliance = DriverStation.getAlliance().get();
  }

  @Override
  public void execute() {

    // boolean runGet = run.get();
    // boolean overrideGet = override.get();

    Pose2d currentPose = pose.get();
    double x = currentPose.getX();
    double y = currentPose.getY();

    boolean inBottomTrench = Zones.isInBlueBottomTrench(x, y) || Zones.isInRedBottomTrench(x, y);

    boolean inTopTrench = Zones.isInBlueTopTrench(x, y) || Zones.isInRedTopTrench(x, y);

    Logger.recordOutput("Zones/x-position", x);
    Logger.recordOutput("Zones/y-position", y);
    Logger.recordOutput("Zones/bottomtrench", inBottomTrench);
    Logger.recordOutput("Zones/toptrench", inTopTrench);

    if (inBottomTrench || inTopTrench) {
      hood.setHoodPos(0.0);
    } else {
      hood.setHoodPos(0.3);
    }
  }

  public boolean getHublock() {
    return hubLock;
  }
}

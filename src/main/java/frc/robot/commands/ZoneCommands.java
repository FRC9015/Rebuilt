package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Zones;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import java.util.function.Supplier;

public class ZoneCommands extends Command {
  private final Drive drive;
  private Supplier<Pose2d> pose;
  private final Hood hood;

  public ZoneCommands(Supplier<Pose2d> pose, Drive drive, Hood hood) {
    this.drive = drive;
    this.pose = pose;
    this.hood = hood;
  }

  @Override
  public void execute() {
    if ((Zones.getCurrentFieldZone(pose) == Zones.FieldZone.BLUE_BOTTOM_TRENCH_DUCK)
        || (Zones.getCurrentFieldZone(pose) == Zones.FieldZone.RED_BOTTOM_TRENCH_DUCK)
        || (Zones.getCurrentFieldZone(pose) == Zones.FieldZone.RED_TOP_TRENCH_DUCK)
        || (Zones.getCurrentFieldZone(pose) == Zones.FieldZone.BLUE_TOP_TRENCH_DUCK)) {
      hood.setHoodPosition(0.0);
    }

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      if (Zones.getCurrentFieldZone(pose) == Zones.FieldZone.BLUE_ALLIANCE) {
        // levi pass comman here
        return;
      }
    }

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      if (Zones.getCurrentFieldZone(pose) == Zones.FieldZone.RED_ALLIANCE) {
        // levi pass comman here
        return;
      }
    }
  }
  //   public Command hoodAngle(Supplier<Pose2d> pose) {
  //     if ((Zones.getCurrentFieldZone(pose) == Zones.FieldZone.BLUE_BOTTOM_TRENCH_DUCK)
  //         || (Zones.getCurrentFieldZone(pose) == Zones.FieldZone.RED_BOTTOM_TRENCH_DUCK)
  //         || (Zones.getCurrentFieldZone(pose) == Zones.FieldZone.RED_TOP_TRENCH_DUCK)
  //         || (Zones.getCurrentFieldZone(pose) == Zones.FieldZone.BLUE_TOP_TRENCH_DUCK)) {
  //       return Commands.runOnce(() -> hood.setHoodPosition(0.0));
  //     }
  //     return Commands.none();
  //   }
}

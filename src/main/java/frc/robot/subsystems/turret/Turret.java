package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.TurretConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Main turret class which runs had functions to move the turret within bounds and updates inputs
 */
public class Turret extends SubsystemBase {
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIO io;

  private final SysIdRoutine sysId;

  public Turret(TurretIO io) {
    this.io = io;

    sysId =
        new SysIdRoutine(
            new Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(4),
                Seconds.of(5),
                (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())),
            new Mechanism((voltage) -> io.setTurretVoltage(voltage.in(Volts)), null, this));
  }

  /**
   * Moves the turret to the fastest version of the target angle (0-360) while strictly staying
   * within the MINROTATION and MAXROTATION bounds.
   *
   * @param targetAngleDegrees The target angle in a 0-360 circle.
   */
  public double setTurretAngleFastestPath(double targetAngleDegrees) {
    // 1. Get bounds and current position in degrees
    double minDegrees = TurretConstants.MINROTATION * 360.0;
    double maxDegrees = TurretConstants.MAXROTATION * 360.0;
    double currentPosDegrees = inputs.turretResolvedPositionDegrees;

    // 2. Normalize user input to [0, 360)
    double target = targetAngleDegrees % 360.0;
    if (target < 0) target = 0;
    // 3. Generate the two possible absolute destinations
    double option1 = target; // e.g. 90
    double option2 = target - 360.0; // e.g. -270

    // 4. Check which options are physically reachable
    boolean opt1Valid = (option1 >= minDegrees && option1 <= maxDegrees);
    boolean opt2Valid = (option2 >= minDegrees && option2 <= maxDegrees);

    double chosenDegrees;
    if (opt1Valid && opt2Valid) {
      // Both are within bounds, pick the one that is closer
      if (Math.abs(option1 - currentPosDegrees) <= Math.abs(option2 - currentPosDegrees)) {
        chosenDegrees = option1;
      } else {
        chosenDegrees = option2;
      }
    } else if (opt1Valid) {
      // Only option 1 is safe
      chosenDegrees = option1;
    } else if (opt2Valid) {
      // Only option 2 is safe
      chosenDegrees = option2;
    } else {
      // Neither is valid (this happens if your range is narrower than 360 total degrees)
      // Fallback: Pick option 1 but clamp it to the turret limits
      chosenDegrees = MathUtil.clamp(option1, minDegrees, maxDegrees);
    }
    // 5. Send the safe, optimized position to the motor
    // (The setTurretPosition method in IO expects degrees based on your last file)

    Logger.recordOutput("Turret/option1", option1);
    Logger.recordOutput("Turret/option2", option2);

    return chosenDegrees;
  }

  public Command setTurretAngleFastestPathCommand(double targetAngleDegrees) {
    double setpoint = setTurretAngleFastestPath(targetAngleDegrees);
    return this.run(
        () -> {
          io.setTurretPosition(setpoint);
          Logger.recordOutput("Turret/chosenDegrees", setpoint);
        });
  }

  public Command setTurretVoltageCommand(double voltage) {
    return this.startEnd(() -> setTurretVoltage(voltage), () -> io.stop());
  }

  public void setTurretVoltage(double voltage) {
    io.setTurretVoltage(voltage);
  }

  public Command quasistatic(Direction dir) {
    return sysId.quasistatic(dir);
  }

  public Command dynamic(Direction dir) {
    return sysId.dynamic(dir);
  }

  public Command runSysId() {
    return Commands.sequence(
        sysId.quasistatic(Direction.kForward).withTimeout(4.5),
        sysId.quasistatic(Direction.kReverse).withTimeout(4.5),
        Commands.waitSeconds(3),
        sysId.dynamic(Direction.kForward),
        Commands.waitSeconds(3),
        sysId.dynamic(Direction.kReverse));
  }

  public void setTurretSetPoint(double value) {
    io.setTurretSetPoint(value);
  }

  public boolean turretOutOfRange() {
    return (inputs.turretResolvedPosition > 0.68);
  }

  public boolean turretOutOfRangeneg() {
    return (inputs.turretResolvedPosition < -0.68);
  }

  public double getTurretPositionRadians() {
    return Units.degreesToRadians(inputs.turretResolvedPositionDegrees);
  }

  public void setPositionVoid(double angle) {
    io.setTurretPosition(angle);
  }

  public double getTurretError() {
    return inputs.turretError;
  }

  public Pose2d getTurretPose(Supplier<Translation2d> robotTranslation) {
    return new Pose2d(
        robotTranslation.get(),
        new Rotation2d(getTurretPositionRadians()).rotateBy(new Rotation2d(Math.PI)));
  }

  public void setDriveSetpoint(Rotation2d rot) {
    inputs.driveSetpoint = rot;
  }

  public Rotation2d getDriveSetpoint() {
    return inputs.driveSetpoint;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (!inputs.isZeroed && inputs.hallEffectTriggered) {
      io.seedMotorPosition(TurretConstants.TURRET_ANGLE_OFFSET);
      inputs.isZeroed = true;
    }
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/setpoint", inputs.turretSetpoint);
  }
}

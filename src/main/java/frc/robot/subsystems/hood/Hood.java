package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final InterpolatingTreeMap<Double, Double> launchHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  private final HoodIO io;

  public Hood(HoodIO io) {
    this.io = io;
    launchHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0).getRotations());
    launchHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0).getRotations());
    launchHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0).getRotations());
    launchHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0).getRotations());
    launchHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0).getRotations());
    launchHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0).getRotations());
    launchHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0).getRotations());
    launchHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0).getRotations());
    launchHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0).getRotations());
    launchHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0).getRotations());
  }

  public Command stopHood() {
    return this.run(() -> io.stopHood());
  }

  public Command setHoodPosition(double position) {
    return this.run(() -> io.setHoodPosition(position));
  }

  public Command setInterpolatedPosition(double distance) {
    return this.run(
        () -> {
          double target = launchHoodAngleMap.get(distance);
          io.setHoodPosition(target);
          Logger.recordOutput("Hood/TargetPosition", target);
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public Angle getLaunchAngle() {
    return inputs.launchAngle;
  }

  public double returnHoodSetpoint() {
    return inputs.hoodTargetPosition;
  }

  public double getInterpolatedPosition(double distance) {
    return launchHoodAngleMap.get(distance);
  }
}

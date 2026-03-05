package frc.robot.subsystems.hood;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private InterpolatingTreeMap<Double, Double> hoodInterpolation =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  private final HoodIO io;

  public Hood(HoodIO io) {
    this.io = io;
    hoodInterpolation.put(0.0, 0.0);
    hoodInterpolation.put(0.24, 0.4);
    hoodInterpolation.put(0.48, 1.0);
    hoodInterpolation.put(1.3, 2.8);
  }

  public Command stopHood() {
    return this.run(() -> io.stopHood());
  }

  public Command setHoodPosition(double position) {
    return this.run(() -> io.setHoodPosition(position));
  }

  public Command setInterpolatedPosition(double distance) {
    return this.runOnce(
        () -> {
          double target = hoodInterpolation.get(distance);
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

  public double getInterpolatedPosition(double distance) {
    return hoodInterpolation.get(distance);
  }
}

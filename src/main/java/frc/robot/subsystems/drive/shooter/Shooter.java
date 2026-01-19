package frc.robot.subsystems.drive.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

public class Shooter {

  public final SparkFlex shooterMotor;
  public final RelativeEncoder shooterEncoder;
  public final SparkMaxConfig shooterConfig;
  public final SparkClosedLoopController shooterPIDController;

  public Shooter(int motorID) {
    this.shooterMotor = new SparkFlex(motorID, SparkFlex.MotorType.kBrushless);
    this.shooterEncoder = shooterMotor.getEncoder();
    this.shooterPIDController = shooterMotor.getClosedLoopController();
    this.shooterConfig = new SparkMaxConfig();

    shooterConfig.inverted(true).idleMode(IdleMode.kBrake);
    shooterConfig.encoder.positionConversionFactor(60).velocityConversionFactor(1);
    shooterConfig
        .closedLoop
        .pid(2, 0.0, 0.0)
        .outputRange(0, 1)
        .velocityFF(1.0 / 565.0); // Value received from REV docs:
    // https://docs.revrobotics.com/sites-test-ion/rev-ion-brushless/revlib/closed-loop-control-overview/closed-loop-control-getting-started

    shooterMotor.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void shootForward(double speed) {
    double newSpeed2 = MathUtil.clamp(speed, 0, 1);
    shooterPIDController.setSetpoint(newSpeed2, SparkFlex.ControlType.kDutyCycle);
  }

  public void shootBackward(double speed) {
    double newSpeed = MathUtil.clamp(speed, 0, 1);
    shooterPIDController.setSetpoint(-newSpeed, SparkFlex.ControlType.kDutyCycle);
  }

  public void stop() {
    shooterMotor.stopMotor();
  }
}

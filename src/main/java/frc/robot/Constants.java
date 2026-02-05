// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class motorIDConstants {
    // placeholders
    public static final int UPPER_INTAKE_MOTOR_ID = 0;
    public static final int EXTEND_INTAKE_MOTOR_ID = 0;
  }

  public static class intakeConstants {
    public static final Slot0Configs intakeSlotPositionConfigs =
        new Slot0Configs()
            .withKP(2)
            .withKI(0)
            .withKD(0.05)
            .withKG(0.01)
            .withKA(0)
            .withKS(0)
            .withKV(0);

    public static final Slot1Configs intakeSlotVelocityConfigs =
        new Slot1Configs().withKP(2).withKI(0).withKD(0).withKG(0).withKA(0).withKS(0).withKV(0);

    public static final MotionMagicConfigs GROUND_MAGIC_CONFIGS =
        new MotionMagicConfigs().withMotionMagicAcceleration(100).withMotionMagicCruiseVelocity(25);

    public static final FeedbackConfigs GROUND_FEEDBACK_CONFIGS =
        new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    public static final double INTAKE_MAX_POS = 300.0;
    public static final double INTAKE_MIN_POS = 0.0;
    public static final double INTAKE_MAX_SPEED = 512.0;
    public static final double INTAKE_MIN_SPEED = -511.0;

    public static final double INTAKE_DEPLOYED_POSITION = 100.0;
    public static final double INTAKE_STOWED_POSITION = 10.0;
  }

  public static class shooterConstants {
    public static final int FlywheelLeftID = 5;
    public static final int FlywheelRightID = 6;
    public static final int HoodID = 7;
    public static final int HoodEncoderID = 8;

    public static final Slot0Configs flyWheelSlotVelocityConfigs =
        new Slot0Configs()
            .withKP(0.01)
            .withKI(0)
            .withKD(0.0025)
            .withKG(0)
            .withKA(0)
            .withKS(0)
            .withKV(0);

    public static final Slot1Configs hoodSlotPositionConfigs =
        new Slot1Configs()
            .withKP(0.02)
            .withKI(0)
            .withKD(0.005)
            .withKG(0.001)
            .withKA(0)
            .withKS(0)
            .withKV(0);

    public static final MotionMagicConfigs HOOD_MAGIC_CONFIGS =
        new MotionMagicConfigs().withMotionMagicAcceleration(100).withMotionMagicCruiseVelocity(25);

    public static final FeedbackConfigs HOOD_FEEDBACK_CONFIGS =
        new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // TODO tune these values

    public static final double SHOOTER_MAX_SPEED = 100;
    public static final double SHOOTER_MIN_SPEED = 0.0;
    public static final double FLYWHEEL_ACCELERATION = 100;
    public static final double FEEDFORWARD_VOLTAGE = 12.0;
    public static final double HOOD_MAX_POS = 0.0;
    public static final double HOOD_MIN_POS = 0.0;
    public static final double HOOD_DEPLOYED_POSITION = 0.0;
    public static final double HOOD_STOWED_POSITION = 0.0;
  }
}

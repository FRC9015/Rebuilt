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
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /** The runtime mode for the robot (real, simulation, or replay). */
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** IDs for motors used by the robot (placeholders — replace with real IDs). */
  public static class MotorIDConstants {
    // placeholders
    public static final int UPPER_INTAKE_MOTOR_ID = 0;
    public static final int EXTEND_INTAKE_MOTOR_ID = 0;
    public static final int TURRET_MOTOR_ID = 1;
    public static final int INDEXER_MOTOR_ID = 13;
  }

  public static class IntakeConstants {
    public static final Slot0Configs intakeSlotPositionConfigs =
        new Slot0Configs()
            .withKP(0.001)
            .withKI(0)
            .withKD(0.02)
            .withKG(0.01)
            .withKA(0)
            .withKS(0)
            .withKV(0);

    public static final int INTAKE_MOTOR_ID = 27;
    public static final int INTAKE2_MOTOR_ID = 28;

    public static final Slot1Configs intakeSlotVelocityConfigs =
        new Slot1Configs().withKP(2).withKI(0).withKD(0).withKG(0).withKA(0).withKS(0).withKV(0);

    public static final MotionMagicConfigs GROUND_MAGIC_CONFIGS =
        new MotionMagicConfigs().withMotionMagicAcceleration(100).withMotionMagicCruiseVelocity(25);

    public static final FeedbackConfigs GROUND_FEEDBACK_CONFIGS =
        new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    public static final double INTAKE_MAX_SPEED = 512.0;
    public static final double INTAKE_MIN_SPEED = -511.0;

    public static final double PIVOT_MAX_POS = 300.0;
    public static final double PIVOT_MIN_POS = 0.0;
    public static final double PIVOT_DEPLOYED_POSITION = 100.0;
    public static final double PIVOT_STOWED_POSITION = 10.0;
  }

  public static class SimConstants {
    // Simulation constants (e.g., physics parameters) can be added here
    public static final double INTAKE_LENGTH = 0.2;
    public static final double INTAKE_WIDTH = 0.7;
    public static final int HOPPER_CAPACITY = 50;
    public static final String GAMEPIECE = "Fuel";
  }

  public static class turretConstants {
    // --- GEAR TEETH ---
    public static final int T_TEETH = 90; // Gear count on final turret gear
    public static final int E1_TEETH = 13; // Gear on Encoder 1
    public static final int E2_TEETH = 15; // Gear on Encoder 2

    public static final int ENCODER_13_TOOTH = 35; // Encoder 13 motor id
    public static final int ENCODER_15_TOOTH = 36; // Encoder 15 motor id

    // --- MATH CONSTANTS ---
    /** The error allowance (in turret rotations) when comparing encoder predictions. */
    public static final double CRT_TOLERANCE = 0.034;

    /**
     * The difference threshold between calculated and internal motor position to trigger a re-seed.
     */
    public static final double SYNC_THRESHOLD = 0.05;

    /** Search limit for Encoder 1 (should be equal to e2_teeth). */
    public static final int E1_SEARCH_LIMIT = (int) E2_TEETH;
    /** Search limit for Encoder 2 (should be equal to e1_teeth). */
    public static final int E2_SEARCH_LIMIT = (int) E1_TEETH;

    // --- MOVEMENT LIMITS ---
    public static final double MAXROTATION = 1.0;
    public static final double MINROTATION = -1.0;

    public static final double ENCODER13_MAGNET_OFFSET = -0.1020507;
    public static final double ENCODER15_MAGNET_OFFSET = 0.1274414;

    // total gear ratio on turret
    public static final double ENCODER_TO_TURRET_GEAR_RATIO = 37.5;
    // --- MOTOR CONFIGS ---
    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS =
        new MotionMagicConfigs().withMotionMagicAcceleration(150).withMotionMagicCruiseVelocity(50);
    public static final Slot0Configs SLOT0_CONFIGS =
        new Slot0Configs() // TODO Tune these values
            .withKP(6)
            .withKI(0.01)
            .withKD(0.2)
            .withKG(0)
            .withKA(0)
            .withKS(0)
            .withKV(0);

    public static final FeedbackConfigs FEEDBACK_CONFIGS =
        new FeedbackConfigs()
            .withSensorToMechanismRatio(ENCODER_TO_TURRET_GEAR_RATIO)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
  }

  public static class FieldConstants {
    public static final Pose2d HUB_POSE_BLUE =
        new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.845), new Rotation2d());
    public static final Pose2d HUB_POSE_RED = FlippingUtil.flipFieldPose(HUB_POSE_BLUE);

    public static final Slot1Configs hoodSlotPositionConfigs =
        new Slot1Configs()
            .withKP(0.02)
            .withKI(0)
            .withKD(0.005)
            .withKG(0.001)
            .withKA(0)
            .withKS(0)
            .withKV(0);

    public static final MotionMagicConfigs hoodMagicConfigs =
        new MotionMagicConfigs().withMotionMagicAcceleration(100).withMotionMagicCruiseVelocity(25);

    public static final FeedbackConfigs hoodFeedbackConfigs =
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
    public static class ShooterConstants {
    public static final int FLY_WHEEL_LEFT_ID = 5;
    public static final int FLY_WHEEL_RIGHT_ID = 29;
    public static final int HOOD_ID = 3;
    public static final int HOOD_ENCODER_ID = 11;

    public static final Slot0Configs flyWheelSlotVelocityConfigs =
        new Slot0Configs()
            .withKP(0.01)
            .withKI(0)
            .withKD(0.0025)
            .withKG(0)
            .withKA(0)
            .withKS(0)
            .withKV(0);
    }
}

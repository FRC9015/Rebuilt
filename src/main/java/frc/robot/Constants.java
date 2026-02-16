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
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    public static final int PIVOT_MOTOR_ID = 28;

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

  public static class ShooterConstants {
    public static final int FlywheelLeftID = 5;
    public static final int FlywheelRightID = 31;
    public static final int HoodID = 3;
    public static final int HoodEncoderID = 8;

    public static final Slot0Configs flyWheelSlotVelocityConfigs =
        new Slot0Configs()
            .withKP(0.005)
            .withKI(0)
            .withKD(0.0025)
            .withKG(0)
            .withKA(0)
            .withKS(0)
            .withKV(0);

    public static final Slot0Configs hoodSlotPositionConfigs =
        new Slot0Configs().withKP(2).withKI(0).withKD(0).withKG(0.01).withKA(0).withKS(0).withKV(0);

    public static final MotionMagicConfigs HOOD_MAGIC_CONFIGS =
        new MotionMagicConfigs().withMotionMagicAcceleration(100).withMotionMagicCruiseVelocity(25);

    public static final FeedbackConfigs HOOD_FEEDBACK_CONFIGS =
        new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // TODO tune these values

    public static final double SHOOTER_MAX_SPEED = 6784.0;
    public static final double SHOOTER_MIN_SPEED = 0.0;
    public static final double FLYWHEEL_ACCELERATION = 100;
    public static final double FEEDFORWARD_VOLTAGE = 12.0;
    public static final double HOOD_MAX_POS = 15.5;
    public static final double HOOD_MIN_POS = -0.3;
    public static final double HOOD_DEPLOYED_POSITION = 15.5;
    public static final double HOOD_STOWED_POSITION = -0.3;
  }

  public static class VisionConstants {
    public static final double MAX_AMBIGUITY = 0.1;
    public static final int MAX_AVERAGE_DISTANCE = 3;
    public static final int STD_DEV_RANGE = 30;
  }

  public static class CameraConstants {
    public static final AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final double FIELD_LENGTH = aprilTagLayout.getFieldLength();
    public static final double FIELD_WIDTH = aprilTagLayout.getFieldWidth();
    public static final double cameraHeight = Units.inchesToMeters(7);
    public static final double cameraPitch = Units.degreesToRadians(15);
    public static final double cameraXOffset = Units.inchesToMeters(13.5);
    public static final Transform3d placeHolderCamera =
        new Transform3d(
            new Translation3d(cameraXOffset, Units.inchesToMeters(0), cameraHeight),
            new Rotation3d(0, cameraPitch, Units.degreesToRadians(0)));

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(5, 5, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }
}

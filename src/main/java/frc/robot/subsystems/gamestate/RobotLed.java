package frc.robot.subsystems.gamestate;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RobotLed extends SubsystemBase {
  private static final int NUM_LEDS = 110; // The number of LEDs on the strip
  private final CANdle candle =
      new CANdle(
          Constants.LedConstants
              .CANDLE_ID1); // CANdle is the CTRE hardware we use to configure our LEDs
  private final StrobeAnimation bufferedAnimation = new StrobeAnimation(0, NUM_LEDS);

  // The constructor just configures the CANdle
  public RobotLed() {
    var candleConfigurator = candle.getConfigurator();

    CANdleConfiguration candleConfig = new CANdleConfiguration();
    candleConfig
        .withCANdleFeatures(
            new CANdleFeaturesConfigs()
                .withVBatOutputMode(VBatOutputModeValue.Modulated)
                .withStatusLedWhenActive(StatusLedWhenActiveValue.Disabled))
        .withLED(
            new LEDConfigs()
                .withBrightnessScalar(1.0)
                .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning)
                .withStripType(StripTypeValue.RGB));

    candleConfigurator.apply(candleConfig);
  }
  /** Sets color */
  public void setColor(RGBWColor color) {
    bufferedAnimation.withColor(color).withFrameRate(0);
  }
  /** Sets color and framerate */
  public void strobeAnimation(RGBWColor color) {
    bufferedAnimation
        .withColor(color)
        .withFrameRate(Constants.LedConstants.DEFAULT_STROBE_FRAME_RATE);
  }
  /** Sets color to CANdle */
  public void updateLEDs() {
    candle.setControl(bufferedAnimation);
  }

  @Override
  public void periodic() {
    updateLEDs();
  }
}

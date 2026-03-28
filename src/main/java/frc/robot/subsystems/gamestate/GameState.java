package frc.robot.subsystems.gamestate;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.AutoLogOutput;

public class GameState extends SubsystemBase {
  @AutoLogOutput private StateEnum state = StateEnum.UNKNOWN;

  @AutoLogOutput private boolean isHubActive = true;
  @AutoLogOutput private boolean isThreeSeconds = false;
  @AutoLogOutput private StateEnum alliance = StateEnum.UNKNOWN;
  @AutoLogOutput private boolean allianceResolved = false;

  @AutoLogOutput private String gameDataManual = "";
  private Timer mTimer;

  public GameState() {
    mTimer = new Timer();
    mTimer.reset();
    mTimer.start();
  }

  @Override
  public void periodic() {
    this.state = getGameState();
    if (!allianceResolved) this.alliance = findAlliance();
    this.isHubActive = canWeScore();
    this.isThreeSeconds = threeSecondTime();
  }

  public StateEnum getState() {
    return this.state;
  }

  public boolean getIsHubActive() {
    return this.isHubActive;
  }

  public boolean getisThreeSeconds() {
    return this.isThreeSeconds;
  }

  public StateEnum getAlliance() {
    return this.alliance;
  }

  public boolean getIsAllianceResolved() {
    return this.allianceResolved;
  }

  private void setGameDataManualy(String teamColor) {
    this.gameDataManual = teamColor;
  }

  public Command manualGameData(String color) {
    return runOnce(() -> setGameDataManualy(color));
  }

  /**
   * finds the games current state
   *
   * @return the game's state, A for auto, T for transition, B for blue, R for red, and E for
   *     endgame.
   */
  private StateEnum getGameState() throws NoSuchElementException {
    double time = -1;
    boolean auto = DriverStation.isAutonomous();
    String gameData = this.gameDataManual;
    if (this.gameDataManual == "") gameData = DriverStation.getGameSpecificMessage();
    try {
      time = OptionalDouble.of(DriverStation.getMatchTime()).getAsDouble();
    } catch (Exception noSuchElementException) {
      time = this.mTimer.get();
    }
    if (auto) {
      return StateEnum.AUTO;
    }
    if (time > 130) {
      return StateEnum.TRANSITION;
    } else if (time > 30) {
      if (gameData == null) return StateEnum.UNKNOWN;
      if ("B".equals(gameData)) {
        if (time > 105) {
          return StateEnum.RED_TEAM;
        } else if (time > 80) {
          return StateEnum.BLUE_TEAM;
        } else if (time > 55) {
          return StateEnum.RED_TEAM;
        } else if (time > 30) {
          return StateEnum.BLUE_TEAM;
        }
      }
      if ("R".equals(gameData)) {
        if (time > 105) {
          return StateEnum.BLUE_TEAM;
        } else if (time > 80) {
          return StateEnum.RED_TEAM;
        } else if (time > 55) {
          return StateEnum.BLUE_TEAM;
        } else if (time > 30) {
          return StateEnum.RED_TEAM;
        }
      }
    }
    if (time == -1) {
      return StateEnum.PRACTICE;
    }
    return StateEnum.ENDGAME;
  }
  /**
   * checks if the robot can score
   *
   * @return true if tower enabled, false if tower disabled
   */
  private boolean canWeScore() {
    return !((this.state == StateEnum.BLUE_TEAM || this.state == StateEnum.RED_TEAM)
        && this.state != this.alliance);
  }

  private boolean threeSecondTime() {
    double time = 0.0;
    try {
      time = OptionalDouble.of(DriverStation.getMatchTime()).getAsDouble();
    } catch (Exception noSuchElementException) {
      return false;
    }
    double epsilon = 0.21;
    return Math.abs(time - 136) < epsilon
        || Math.abs(time - 111) < epsilon
        || Math.abs(time - 86) < epsilon
        || Math.abs(time - 61) < epsilon
        || Math.abs(time - 36) < epsilon
        || Math.abs(time - 6) < epsilon;
  }

  private StateEnum findAlliance() {
    Optional<Alliance> alli = DriverStation.getAlliance();
    if (alli.isEmpty()) return StateEnum.BLUE_TEAM;
    Alliance allia = alli.get();
    this.allianceResolved = true;
    if (allia == Alliance.Blue) {
      return StateEnum.BLUE_TEAM;
    } else {
      return StateEnum.RED_TEAM;
    }
  }
}

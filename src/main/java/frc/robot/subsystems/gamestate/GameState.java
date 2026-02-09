package frc.robot.subsystems.gamestate;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.NoSuchElementException;
import java.util.OptionalDouble;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class GameState extends SubsystemBase {
  @AutoLogOutput private StateEnum state = StateEnum.UNKNOWN;

  @AutoLogOutput private boolean isHubActive = true;
  @AutoLogOutput private boolean isThreeSeconds = false;
  private Supplier<Alliance> allianceRaw;
  @AutoLogOutput private StateEnum alliance = StateEnum.UNKNOWN;

  @AutoLogOutput private String gameDataManual = "";
  @AutoLogOutput private Timer mTimer;

  public GameState(Supplier<Alliance> alliance) {
    mTimer.reset();
    mTimer.start();
    this.allianceRaw = alliance;
  }

  @Override
  public void periodic() {
    this.state = getGameState();
    if (this.alliance == StateEnum.UNKNOWN) this.alliance = findAlliance();
    this.isHubActive = canWeScore();
    this.isThreeSeconds = threeSecondTime();
    System.out.println("state" + state);
    System.out.println("hub" + isHubActive);
    System.out.println("Alliance" + this.alliance);
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

  public void setGameDataManualy(int button) {
    if (button == 1) {
      this.gameDataManual = "B";
    } else {
      this.gameDataManual = "R";
    }
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
    String gameDataManuala = this.gameDataManual;
    if (this.gameDataManual == "") gameDataManuala = DriverStation.getGameSpecificMessage();
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
      if (gameDataManuala == null) return StateEnum.UNKNOWN;
      if ("B".equals(gameDataManuala)) {
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
      if ("R".equals(gameDataManuala)) {
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
    if ((this.state == StateEnum.BLUE_TEAM || this.state == StateEnum.RED_TEAM)
        && this.alliance != StateEnum.PRACTICE
        && this.state != this.alliance) {
      return false;
    } else {
      return true;
    }
  }

  private boolean threeSecondTime() {
    double time = 0.0;
    try {
      time = OptionalDouble.of(DriverStation.getMatchTime()).getAsDouble();
    } catch (Exception noSuchElementException) {
      return false;
    }
    double epsilon = 0.21;
    return Math.abs(time - 133) < epsilon
        || Math.abs(time - 108) < epsilon
        || Math.abs(time - 83) < epsilon
        || Math.abs(time - 58) < epsilon
        || Math.abs(time - 33) < epsilon
        || Math.abs(time - 3) < epsilon;
  }

  private StateEnum findAlliance() {
    Alliance allia = this.allianceRaw.get();
    if (allia == Alliance.Blue) {
      return StateEnum.BLUE_TEAM;
    } else if (allia == Alliance.Red) {
      return StateEnum.RED_TEAM;
    } else {
      return StateEnum.PRACTICE;
    }
  }
}

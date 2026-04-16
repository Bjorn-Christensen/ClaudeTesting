// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Setter;

public class HubShiftUtil {
  public enum ShiftEnum {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED;
  }

  public record ShiftInfo(
      ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

  private static Timer shiftTimer = new Timer();
  private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

  private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
  private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

  private static final double minFuelCountDelay = 1.0;
  private static final double maxFuelCountDelay = 2.0;
  private static final double shiftEndFuelCountExtension = 3.0;
  private static final double approachingActiveFudge = -1 * (minFuelCountDelay);
  private static final double endingActiveFudge =
      shiftEndFuelCountExtension + -1 * (maxFuelCountDelay);

  public static final double autoEndTime = 20.0;
  public static final double teleopDuration = 140.0;
  private static final boolean[] activeSchedule = {true, true, false, true, false, true};
  private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};
  private static final double timeResetThreshold = 3.0;
  private static double shiftTimerOffset = 0.0;
  @Setter private static Supplier<Optional<Boolean>> allianceWinOverride = () -> Optional.empty();

  private static GenericEntry ourHubActiveEntry;
  private static GenericEntry blueActiveEntry;
  private static GenericEntry redActiveEntry;
  private static GenericEntry currentShiftEntry;
  private static GenericEntry shiftRemainingEntry;

  public static void initShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Hub Shift");

    ourHubActiveEntry =
        tab.add("Our Hub Active", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(4, 2)
            .withSize(2, 2)
            .getEntry();

    blueActiveEntry =
        tab.add("Blue", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "blue", "Color when false", "#222222"))
            .withPosition(2, 0)
            .withSize(2, 2)
            .getEntry();

    redActiveEntry =
        tab.add("Red", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "red", "Color when false", "#222222"))
            .withPosition(6, 0)
            .withSize(2, 2)
            .getEntry();

    currentShiftEntry =
        tab.add("Current Shift", "DISABLED")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 1)
            .withSize(2, 1)
            .getEntry();

    shiftRemainingEntry =
        tab.add("Shift Remaining (s)", "0.00")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 0)
            .withSize(2, 1)
            .getEntry();
  }

  public static void updateShuffleboard() {
    ShiftInfo official = getOfficialShiftInfo();
    Alliance myAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Alliance currentlyActive =
        official.active()
            ? myAlliance
            : (myAlliance == Alliance.Blue ? Alliance.Red : Alliance.Blue);

    ourHubActiveEntry.setBoolean(official.active());
    blueActiveEntry.setBoolean(currentlyActive == Alliance.Blue);
    redActiveEntry.setBoolean(currentlyActive == Alliance.Red);
    currentShiftEntry.setString(official.currentShift().name());
    shiftRemainingEntry.setString(String.format("%.2f", official.remainingTime()));
  }

  public static Optional<Boolean> getAllianceWinOverride() {
    return allianceWinOverride.get();
  }

  public static Alliance getFirstActiveAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Return override value
    var winOverride = getAllianceWinOverride();
    if (!winOverride.isEmpty()) {
      return winOverride.get()
          ? (alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue)
          : (alliance == Alliance.Blue ? Alliance.Blue : Alliance.Red);
    }

    // Return FMS value
    String message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      char character = message.charAt(0);
      if (character == 'R') {
        return Alliance.Blue;
      } else if (character == 'B') {
        return Alliance.Red;
      }
    }

    // Return default value
    return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }

  /** Starts the timer at the begining of teleop. */
  public static void initialize() {
    shiftTimerOffset = 0;
    shiftTimer.restart();
  }

  private static boolean[] getSchedule() {
    boolean[] currentSchedule;
    Alliance startAlliance = getFirstActiveAlliance();
    currentSchedule =
        startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
            ? activeSchedule
            : inactiveSchedule;
    return currentSchedule;
  }

  private static ShiftInfo getShiftInfo(
      boolean[] currentSchedule, double[] shiftStartTimes, double[] shiftEndTimes) {
    double timerValue = shiftTimer.get();
    double currentTime = timerValue - shiftTimerOffset;
    double stateTimeElapsed = currentTime;
    double stateTimeRemaining = 0.0;
    boolean active = false;
    ShiftEnum currentShift = ShiftEnum.DISABLED;
    double fieldTeleopTime = 140.0 - DriverStation.getMatchTime();

    if (DriverStation.isAutonomousEnabled()) {
      stateTimeElapsed = currentTime;
      stateTimeRemaining = autoEndTime - currentTime;
      active = true;
      currentShift = ShiftEnum.AUTO;
    } else if (DriverStation.isEnabled()) {
      // Adjust the current offset if the time difference above the theshold
      if (Math.abs(fieldTeleopTime - currentTime) >= timeResetThreshold
          && fieldTeleopTime <= 135
          && DriverStation.isFMSAttached()) {
        shiftTimerOffset += currentTime - fieldTeleopTime;
        currentTime = timerValue + shiftTimerOffset;
      }
      int currentShiftIndex = -1;
      for (int i = 0; i < shiftStartTimes.length; i++) {
        if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
          currentShiftIndex = i;
          break;
        }
      }
      if (currentShiftIndex < 0) {
        // After last shift, so assume endgame
        currentShiftIndex = shiftStartTimes.length - 1;
      }

      // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
      stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
      stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

      // If the state is the same as the last shift, combine the elapsed time
      if (currentShiftIndex > 0) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
          stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
        }
      }

      // If the state is the same as the next shift, combine the remaining time
      if (currentShiftIndex < shiftEndTimes.length - 1) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
          stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
        }
      }

      active = currentSchedule[currentShiftIndex];
      currentShift = shiftsEnums[currentShiftIndex];
    }
    ShiftInfo shiftInfo = new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
    return shiftInfo;
  }

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
  }

  public static ShiftInfo getShiftedShiftInfo() {
    boolean[] shiftSchedule = getSchedule();
    // Starting active
    if (shiftSchedule[1] == true) {
      double[] shiftedShiftStartTimes = {
        0.0,
        10.0,
        35.0 + endingActiveFudge,
        60.0 + approachingActiveFudge,
        85.0 + endingActiveFudge,
        110.0 + approachingActiveFudge
      };
      double[] shiftedShiftEndTimes = {
        10.0,
        35.0 + endingActiveFudge,
        60.0 + approachingActiveFudge,
        85.0 + endingActiveFudge,
        110.0 + approachingActiveFudge,
        140.0
      };
      return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
    }
    double[] shiftedShiftStartTimes = {
      0.0,
      10.0 + endingActiveFudge,
      35.0 + approachingActiveFudge,
      60.0 + endingActiveFudge,
      85.0 + approachingActiveFudge,
      110.0
    };
    double[] shiftedShiftEndTimes = {
      10.0 + endingActiveFudge,
      35.0 + approachingActiveFudge,
      60.0 + endingActiveFudge,
      85.0 + approachingActiveFudge,
      110.0,
      140.0
    };
    return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
    // }
  }
}
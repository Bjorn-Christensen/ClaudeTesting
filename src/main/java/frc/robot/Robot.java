// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.util.HubShiftUtil;

public class Robot extends LoggedRobot {
  
  private RobotContainer robotContainer;
  private Command autonomousCommand;
  private Timer disabledTimer;

  @Override
  public void robotInit() {
    // Instantiate the RobotContainer
    robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable
    disabledTimer = new Timer();

    // Cuts out unnecessary error logs when simulating robot
    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    // AdvantageKit setup
    Logger.addDataReceiver(new NT4Publisher()); // live stream to AdvantageScope
    Logger.start();

    HubShiftUtil.initShuffleboard();
  }

  @Override
  public void robotPeriodic() {
    // Required to run command based code
    CommandScheduler.getInstance().run();
    HubShiftUtil.updateShuffleboard();
  }

  @Override
  public void disabledInit() {
    robotContainer.setDriveMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(DrivetrainConstants.WHEEL_LOCK_TIME)) {
      robotContainer.setDriveMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  @Override
  public void autonomousInit() {
    robotContainer.setDriveMotorBrake(true);
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {

    // This makes sure that the autonomous stops running when teleop starts running
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }

    // Set default commands once the match begins
    robotContainer.setSwerveDefaultCommand();
    HubShiftUtil.initialize();
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {

    // Cancels all running commands at the start of test mode
    CommandScheduler.getInstance().cancelAll();

    // Set default commands once the match begins
    robotContainer.setSwerveDefaultCommand();
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void simulationInit() {

  }

  @Override
  public void simulationPeriodic() {
    
  }
}

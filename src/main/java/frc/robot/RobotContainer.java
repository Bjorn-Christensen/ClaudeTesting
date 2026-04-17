package frc.robot;

import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;

import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import swervelib.SwerveInputStream;

public class RobotContainer {

  // Subsystems
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(DrivetrainConstants.swerveJsonDirectory);
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeRollerSubsystem intakeRollerSubsystem = new IntakeRollerSubsystem();
  private final IntakePivotSubsystem intakePivotSubsystem = new IntakePivotSubsystem();

  // Joysticks
  final CommandXboxController controllerXbox    = new CommandXboxController(0); // driver
  final CommandXboxController operatorXbox      = new CommandXboxController(1); // operator

  // Auton Chooser
  private final LoggedDashboardChooser<Command> autoChooser;

  // Subsystem requirements for deferred commands (required by Commands.defer)
  private final Set<Subsystem> swerveReq   = Set.of(swerveSubsystem);
  private final Set<Subsystem> shooterReq  = Set.of(shooterSubsystem);

  // Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                () -> controllerXbox.getLeftY() * -1,
                                                                () -> controllerXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> controllerXbox.getRightX() * -1)
                                                            .deadband(ControllerConstants.DEADZONE)
                                                            .scaleTranslation(0.8)
                                                            .scaleRotation(0.8)
                                                            .allianceRelativeControl(true);

  // Constructor
  public RobotContainer() {
    configureButtonBindings();

    // Register named commands for PathPlanner
    NamedCommands.registerCommand("lineUpShot",
        Commands.defer(() -> swerveSubsystem.lineUpForShooter(), swerveReq));
    NamedCommands.registerCommand("deployIntake",  intakePivotSubsystem.deployPivotCommand());
    NamedCommands.registerCommand("retractIntake", intakePivotSubsystem.retractPivotCommand());
    NamedCommands.registerCommand("runIntake",     intakeRollerSubsystem.intakeCommand());
    NamedCommands.registerCommand("stopIntake",    intakeRollerSubsystem.stopCommand());

    // Shoot for a fixed duration using the range table and isFacingGoal as the ready gate.
    // Commands.defer() re-evaluates the tunable values each time the command is scheduled.
    NamedCommands.registerCommand("shootBalls",
        Commands.defer(() -> shooterSubsystem.shootOnReadyCommand(
            ShooterConstants.DEFAULT_FLYWHEEL_RPM,
            ShooterConstants.DEFAULT_BACK_ROLLER_RPM,
            ShooterConstants.DEFAULT_FEED_RPM,
            swerveSubsystem::isFacingGoal
        ).withTimeout(ShooterConstants.SHOOT_DURATION_SECONDS), shooterReq));

    // Build chooser after NamedCommands are registered so that event markers have something to call
    autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser("Test"));
  }

  private void configureButtonBindings() {

    // ---- Driver controller (port 0) — driving & shooting (triggers) ----

    // Auto line-up to shooting position
    controllerXbox.povUp().onTrue(Commands.defer(() -> swerveSubsystem.lineUpForShooter(), swerveReq));

    // Reset odometry to (0,0) — useful during testing/practice, never press mid-match
    controllerXbox.povLeft().onTrue(Commands.runOnce(() -> swerveSubsystem.resetOdometry(new Pose2d()), swerveSubsystem));

    // Hold right trigger for precision mode — reduces translation and rotation to SLOW_MODE_SCALE
    // for fine shot lineup adjustments. Releases back to normal drive automatically.
    controllerXbox.rightTrigger(ControllerConstants.RIGHT_TRIGGER_DEADZONE).whileTrue(
        swerveSubsystem.driveFieldOriented(
            SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                 () -> controllerXbox.getLeftY() * -1,
                                 () -> controllerXbox.getLeftX() * -1)
                            .withControllerRotationAxis(() -> controllerXbox.getRightX() * -1)
                            .deadband(ControllerConstants.DEADZONE)
                            .scaleTranslation(ControllerConstants.SLOW_MODE_SCALE)
                            .scaleRotation(ControllerConstants.SLOW_MODE_SCALE)
                            .allianceRelativeControl(true)));

    // Hold left trigger to test-shoot — range-adjusted RPM, no facing-goal check.
    // Use when cameras are off or during bench/distance testing.
    controllerXbox.leftTrigger(ControllerConstants.LEFT_TRIGGER_DEADZONE).whileTrue(
        Commands.defer(() -> shooterSubsystem.shootOnReadyCommand(
            ShooterConstants.DEFAULT_FLYWHEEL_RPM,
            ShooterConstants.DEFAULT_BACK_ROLLER_RPM,
            ShooterConstants.DEFAULT_FEED_RPM,
            () -> true
        ), shooterReq)
    );


    // ---- Operator controller (port 1) — intake ----

    // Hold left trigger to run intake rollers inward
    operatorXbox.leftTrigger(ControllerConstants.LEFT_TRIGGER_DEADZONE)
        .whileTrue(intakeRollerSubsystem.intakeCommand());

    // Hold right trigger to eject / reverse intake rollers
    operatorXbox.rightTrigger(ControllerConstants.RIGHT_TRIGGER_DEADZONE)
        .whileTrue(intakeRollerSubsystem.ejectCommand());

    // Hold left bumper to deploy pivot; releasing coasts to bumper stop
    operatorXbox.leftBumper().whileTrue(intakePivotSubsystem.deployPivotCommand());

    // Hold right bumper to retract pivot; release to stop
    operatorXbox.rightBumper().whileTrue(intakePivotSubsystem.retractPivotCommand());

    // Auto-unjam: if roller current spikes for 100ms, briefly retract then redeploy the pivot
    new Trigger(intakeRollerSubsystem::isJammed)
        .debounce(0.1)
        .onTrue(intakePivotSubsystem.unjamCommand());

  }

  // Auton chooser called on Autonomous Init
  public Command getAutonomousCommand() {
    Command selected = autoChooser.get();
    if(selected != null) {
      String autoName = selected.getName();
      try {
        var paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
        if (!paths.isEmpty()) {
          Pose2d startPose = paths.get(0).getPathPoses().get(0);
          swerveSubsystem.primeStartingPose(startPose);
        }
      } catch (Exception e) {
        e.printStackTrace();
      }
    }

    return selected;
  }

  // Set drive Controls For Teleop
  public void setSwerveDefaultCommand() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driveAngularVelocity));
  }

  // Set drive motor brakes
  public void setDriveMotorBrake(boolean brake) {
    swerveSubsystem.setMotorBrake(brake);
    if(!brake) swerveSubsystem.getSwerveDrive().lockPose();
  }

}

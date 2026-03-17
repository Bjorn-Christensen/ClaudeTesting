package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex feedMotor;
    private final SparkFlex flywheelMotor;
    private final SparkFlex backRollerMotor;

    private final SparkClosedLoopController feedController;
    private final SparkClosedLoopController flywheelController;
    private final SparkClosedLoopController backRollerController;

    private double targetFlywheelRpm = 0.0;
    private double targetFeedRpm     = 0.0;

    public ShooterSubsystem() {
        feedMotor        = new SparkFlex(ShooterConstants.FEED_MOTOR_ID,        MotorType.kBrushless);
        flywheelMotor    = new SparkFlex(ShooterConstants.FLYWHEEL_MOTOR_ID,    MotorType.kBrushless);
        backRollerMotor  = new SparkFlex(ShooterConstants.BACK_ROLLER_MOTOR_ID, MotorType.kBrushless);

        // Feed motor — positive output moves ball toward the flywheel
        SparkFlexConfig feedConfig = new SparkFlexConfig();
        feedConfig.inverted(false);
        feedConfig.idleMode(IdleMode.kCoast);
        feedConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.FEED_KP)
            .i(ShooterConstants.FEED_KI)
            .d(ShooterConstants.FEED_KD);
        feedConfig.closedLoop.feedForward.kV(ShooterConstants.FEED_KV);
        feedMotor.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Flywheel motor — inverted relative to feed so it spins in the opposite
        // physical direction, launching the ball out of the shooter hood
        SparkFlexConfig flywheelConfig = new SparkFlexConfig();
        flywheelConfig.inverted(true);
        flywheelConfig.idleMode(IdleMode.kCoast);
        flywheelConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.FLYWHEEL_KP)
            .i(ShooterConstants.FLYWHEEL_KI)
            .d(ShooterConstants.FLYWHEEL_KD);
        flywheelConfig.closedLoop.feedForward.kV(ShooterConstants.FLYWHEEL_KV);
        flywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Back roller motor — inverted relative to the flywheel (same physical
        // direction as the feed). Contacts the back face of the ball and applies
        // an equal and opposite surface velocity to the flywheel contact, cancelling
        // net ball spin for improved shot consistency.
        SparkFlexConfig backRollerConfig = new SparkFlexConfig();
        backRollerConfig.inverted(false);
        backRollerConfig.idleMode(IdleMode.kCoast);
        backRollerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.BACK_ROLLER_KP)
            .i(ShooterConstants.BACK_ROLLER_KI)
            .d(ShooterConstants.BACK_ROLLER_KD);
        backRollerConfig.closedLoop.feedForward.kV(ShooterConstants.BACK_ROLLER_KV);
        backRollerMotor.configure(backRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feedController       = feedMotor.getClosedLoopController();
        flywheelController   = flywheelMotor.getClosedLoopController();
        backRollerController = backRollerMotor.getClosedLoopController();
    }

    // ---- State setters ----

    /** Spin up the flywheel and back roller to the given RPM. */
    public void setFlywheelRpm(double rpm) {
        targetFlywheelRpm = rpm;
        flywheelController.setSetpoint(rpm, ControlType.kVelocity);
        backRollerController.setSetpoint(rpm * ShooterConstants.BACK_ROLLER_SPEED_RATIO, ControlType.kVelocity);
    }

    /** Drive the feed motor at the given RPM. */
    public void setFeedRpm(double rpm) {
        targetFeedRpm = rpm;
        feedController.setSetpoint(rpm, ControlType.kVelocity);
    }

    /** Stop all three shooter motors immediately. */
    public void stopAll() {
        targetFlywheelRpm = 0.0;
        targetFeedRpm     = 0.0;
        flywheelMotor.stopMotor();
        backRollerMotor.stopMotor();
        feedMotor.stopMotor();
    }

    // ---- Queries ----

    /** Returns true when the flywheel is within tolerance of its target RPM. */
    public boolean flywheelAtSpeed() {
        double actual = flywheelMotor.getEncoder().getVelocity();
        return targetFlywheelRpm > 0
            && Math.abs(actual - targetFlywheelRpm) < ShooterConstants.FLYWHEEL_RPM_TOLERANCE;
    }

    // ---- Commands ----

    /**
     * Spin up the flywheel and back roller to the target RPM.
     * Stops both when the command is interrupted.
     * Compose with feedCommand() once flywheelAtSpeed() is true.
     *
     * <pre>
     *   shooterSubsystem.spinUpCommand(ShooterConstants.DEFAULT_FLYWHEEL_RPM)
     *       .alongWith(Commands.waitUntil(shooterSubsystem::flywheelAtSpeed)
     *           .andThen(shooterSubsystem.feedCommand(ShooterConstants.DEFAULT_FEED_RPM)));
     * </pre>
     */
    public Command spinUpCommand(double rpm) {
        return runEnd(
            () -> setFlywheelRpm(rpm),
            () -> {
                flywheelMotor.stopMotor();
                backRollerMotor.stopMotor();
                targetFlywheelRpm = 0.0;
            }
        );
    }

    /** Run the feed motor at the given RPM; stops when interrupted. */
    public Command feedCommand(double rpm) {
        return runEnd(
            () -> setFeedRpm(rpm),
            () -> {
                feedMotor.stopMotor();
                targetFeedRpm = 0.0;
            }
        );
    }

    /**
     * Hold to shoot with a dynamic RPM supplier (e.g. range table lookup).
     * Spins up immediately; feeds only when flywheel is at speed AND robotReady is true.
     * Feed is actively held off during any RPM dip so every ball sees the same launch speed.
     * Everything stops when the button is released.
     */
    public Command shootOnReadyCommand(DoubleSupplier flywheelRpm, double feedRpm, BooleanSupplier robotReady) {
        return runEnd(
            () -> {
                setFlywheelRpm(flywheelRpm.getAsDouble());
                if (flywheelAtSpeed() && robotReady.getAsBoolean()) {
                    setFeedRpm(feedRpm);
                } else {
                    feedMotor.stopMotor();
                    targetFeedRpm = 0.0;
                }
            },
            this::stopAll
        );
    }

    /** Fixed-RPM overload — delegates to the DoubleSupplier version. */
    public Command shootOnReadyCommand(double flywheelRpm, double feedRpm, BooleanSupplier robotReady) {
        return shootOnReadyCommand(() -> flywheelRpm, feedRpm, robotReady);
    }

    /** Stop all shooter motors (one-shot). */
    public Command stopCommand() {
        return runOnce(this::stopAll);
    }

    // ---- Periodic ----

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/Flywheel/TargetRPM",   targetFlywheelRpm);
        Logger.recordOutput("Shooter/Flywheel/ActualRPM",   flywheelMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/Feed/TargetRPM",       targetFeedRpm);
        Logger.recordOutput("Shooter/Feed/ActualRPM",       feedMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/BackRoller/ActualRPM", backRollerMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/FlywheelAtSpeed",      flywheelAtSpeed());
    }
}

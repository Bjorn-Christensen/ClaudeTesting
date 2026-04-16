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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex flywheelMotor;
    private final SparkFlex backRollerMotor;
    private final SparkFlex feedMotor;
    private final SparkFlex feedFollowerMotor;
    private final SparkFlex agitatorMotor;
    private final SparkFlex agitatorFollowerMotor;

    private final SparkClosedLoopController flywheelController;
    private final SparkClosedLoopController backRollerController;
    private final SparkClosedLoopController feedController;

    private double targetFlywheelRpm    = 0.0;
    private double targetBackRollerRpm  = 0.0;
    private double targetFeedRpm        = 0.0;

    private boolean agitatorActive = false; // true when agitator motor is physically running

    public ShooterSubsystem() {
        flywheelMotor         = new SparkFlex(ShooterConstants.FLYWHEEL_MOTOR_ID,          MotorType.kBrushless);
        backRollerMotor       = new SparkFlex(ShooterConstants.BACK_ROLLER_MOTOR_ID,       MotorType.kBrushless);
        feedMotor             = new SparkFlex(ShooterConstants.FEED_MOTOR_ID,              MotorType.kBrushless);
        feedFollowerMotor     = new SparkFlex(ShooterConstants.FEED_FOLLOWER_MOTOR_ID,     MotorType.kBrushless);
        agitatorMotor         = new SparkFlex(ShooterConstants.AGITATOR_MOTOR_ID,          MotorType.kBrushless);
        agitatorFollowerMotor = new SparkFlex(ShooterConstants.AGITATOR_FOLLOWER_MOTOR_ID, MotorType.kBrushless);

        SparkFlexConfig flywheelConfig = new SparkFlexConfig();
        flywheelConfig.inverted(false);
        flywheelConfig.idleMode(IdleMode.kCoast);
        flywheelConfig.smartCurrentLimit(ShooterConstants.FLYWHEEL_CURRENT_LIMIT);
        flywheelConfig.voltageCompensation(ShooterConstants.VOLTAGE_COMPENSATION);
        flywheelConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.FLYWHEEL_KP)
            .i(ShooterConstants.FLYWHEEL_KI)
            .d(ShooterConstants.FLYWHEEL_KD);
        flywheelConfig.closedLoop.feedForward.kV(ShooterConstants.FLYWHEEL_KV);
        flywheelConfig.closedLoopRampRate(ShooterConstants.FLYWHEEL_RAMP_RATE);
        flywheelConfig.signals
            .primaryEncoderPositionPeriodMs(500)  // position unused; keep velocity fast for PID
            .appliedOutputPeriodMs(20)
            .motorTemperaturePeriodMs(500);
        flywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig backRollerConfig = new SparkFlexConfig();
        backRollerConfig.inverted(true);
        backRollerConfig.idleMode(IdleMode.kCoast);
        backRollerConfig.smartCurrentLimit(ShooterConstants.BACK_ROLLER_CURRENT_LIMIT);
        backRollerConfig.voltageCompensation(ShooterConstants.VOLTAGE_COMPENSATION);
        backRollerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.BACK_ROLLER_KP)
            .i(ShooterConstants.BACK_ROLLER_KI)
            .d(ShooterConstants.BACK_ROLLER_KD);
        backRollerConfig.closedLoop.feedForward.kV(ShooterConstants.BACK_ROLLER_KV);
        backRollerConfig.closedLoopRampRate(ShooterConstants.BACK_ROLLER_RAMP_RATE);
        backRollerConfig.signals
            .primaryEncoderPositionPeriodMs(500)
            .appliedOutputPeriodMs(20)
            .motorTemperaturePeriodMs(500);
        backRollerMotor.configure(backRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig feedConfig = new SparkFlexConfig();
        feedConfig.inverted(true);
        feedConfig.idleMode(IdleMode.kCoast);
        feedConfig.smartCurrentLimit(ShooterConstants.FEED_CURRENT_LIMIT);
        feedConfig.voltageCompensation(ShooterConstants.VOLTAGE_COMPENSATION);
        feedConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.FEED_KP)
            .i(ShooterConstants.FEED_KI)
            .d(ShooterConstants.FEED_KD);
        feedConfig.closedLoop.feedForward.kV(ShooterConstants.FEED_KV);
        feedConfig.closedLoopRampRate(ShooterConstants.FEED_RAMP_RATE);
        feedConfig.signals
            .primaryEncoderPositionPeriodMs(500)
            .appliedOutputPeriodMs(20)
            .motorTemperaturePeriodMs(500);
        feedMotor.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig feedFollowerConfig = new SparkFlexConfig();
        feedFollowerConfig.follow(ShooterConstants.FEED_MOTOR_ID, true);
        feedFollowerConfig.idleMode(IdleMode.kCoast);
        feedFollowerConfig.smartCurrentLimit(ShooterConstants.FEED_CURRENT_LIMIT);
        feedFollowerConfig.signals
            .primaryEncoderPositionPeriodMs(500)
            .primaryEncoderVelocityPeriodMs(500)
            .appliedOutputPeriodMs(500)
            .motorTemperaturePeriodMs(500);
        feedFollowerMotor.configure(feedFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig agitatorConfig = new SparkFlexConfig();
        agitatorConfig.inverted(false);
        agitatorConfig.idleMode(IdleMode.kCoast);
        agitatorConfig.smartCurrentLimit(ShooterConstants.AGITATOR_CURRENT_LIMIT);
        agitatorConfig.signals
            .primaryEncoderPositionPeriodMs(500)
            .primaryEncoderVelocityPeriodMs(500)
            .appliedOutputPeriodMs(20)
            .motorTemperaturePeriodMs(500);
        agitatorMotor.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig agitatorFollowerConfig = new SparkFlexConfig();
        agitatorFollowerConfig.follow(ShooterConstants.AGITATOR_MOTOR_ID, true);
        agitatorFollowerConfig.idleMode(IdleMode.kCoast);
        agitatorFollowerConfig.smartCurrentLimit(ShooterConstants.AGITATOR_CURRENT_LIMIT);
        agitatorFollowerConfig.signals
            .primaryEncoderPositionPeriodMs(500)
            .primaryEncoderVelocityPeriodMs(500)
            .appliedOutputPeriodMs(500)
            .motorTemperaturePeriodMs(500);
        agitatorFollowerMotor.configure(agitatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelController    = flywheelMotor.getClosedLoopController();
        backRollerController  = backRollerMotor.getClosedLoopController();
        feedController        = feedMotor.getClosedLoopController();
    }

    // ---- State setters ----

    public void setFlywheelRpm(double rpm) {
        targetFlywheelRpm = rpm;
        flywheelController.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void setBackRollerRpm(double rpm) {
        targetBackRollerRpm = rpm;
        backRollerController.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void setFeedRpm(double rpm) {
        targetFeedRpm = rpm;
        feedController.setSetpoint(rpm, ControlType.kVelocity);
    }

    /**
     * Stop the agitator immediately and clear the latch.
     */
    public void stopAgitator() {
        if (agitatorActive) {
            agitatorMotor.stopMotor();
            agitatorActive = false;
        }
    }

    public void stopAll() {
        targetFlywheelRpm   = 0.0;
        targetBackRollerRpm = 0.0;
        targetFeedRpm       = 0.0;
        flywheelMotor.stopMotor();
        backRollerMotor.stopMotor();
        feedMotor.stopMotor();
        stopAgitator();
    }

    // ---- Queries ----

    public boolean flywheelAtSpeed() {
        double actual = flywheelMotor.getEncoder().getVelocity();
        return targetFlywheelRpm > 0
            && actual >= (targetFlywheelRpm - ShooterConstants.FLYWHEEL_RPM_TOLERANCE);
    }

    public boolean backRollerAtSpeed() {
        double actual = backRollerMotor.getEncoder().getVelocity();
        return targetBackRollerRpm > 0
            && actual >= (targetBackRollerRpm - ShooterConstants.BACK_ROLLER_RPM_TOLERANCE);
    }

    public boolean feedAtSpeed() {
        double actual = feedMotor.getEncoder().getVelocity();
        return targetFeedRpm > 0
            && actual >= (targetFeedRpm - ShooterConstants.FEED_RPM_TOLERANCE);
    }

    // ---- Commands ----

    /**
     * Hold to shoot. Spins up the flywheel, back roller, and feed immediately.
     * The agitator is held until all three are at speed AND robotReady is true,
     * minimising the time balls spend compressed against the flywheel/backroller/feed
     * before they are actually shot.
     */
    public Command shootOnReadyCommand(double flywheelRpm, double backRollerRpm, double feedRpm, BooleanSupplier robotReady) {
        final Timer spinupTimer = new Timer();
        final Timer unjamTimer = new Timer();
        final boolean[] isUnjamming = {false};
        return new FunctionalCommand(
            // initialize — called once when the command starts
            () -> {
                setFlywheelRpm(flywheelRpm);
                setBackRollerRpm(backRollerRpm);
                setFeedRpm(feedRpm);
                spinupTimer.restart();
                isUnjamming[0] = false;
            },
            // execute — fires when at-speed, or after timeout; auto-unjams if agitator jams
            () -> {
                boolean atSpeed = flywheelAtSpeed() && backRollerAtSpeed() && feedAtSpeed();
                boolean timedOut = spinupTimer.hasElapsed(ShooterConstants.AGITATOR_SPINUP_TIMEOUT);

                if (isUnjamming[0]) {
                    // Backroll until unjam duration elapses, then resume
                    agitatorMotor.set(-ShooterConstants.AGITATOR_SPEED);
                    agitatorActive = false;
                    if (unjamTimer.hasElapsed(ShooterConstants.AGITATOR_UNJAM_DURATION)) {
                        isUnjamming[0] = false;
                    }
                } else if ((atSpeed || timedOut) && robotReady.getAsBoolean()) {
                    // Check for jam before running forward
                    if (agitatorActive
                            && agitatorMotor.getOutputCurrent() > ShooterConstants.AGITATOR_JAM_CURRENT) {
                        isUnjamming[0] = true;
                        unjamTimer.restart();
                    } else {
                        agitatorMotor.set(ShooterConstants.AGITATOR_SPEED);
                        agitatorActive = true;
                    }
                } else {
                    agitatorMotor.stopMotor();
                    agitatorActive = false;
                }
            },
            // end — called once when the button is released (or command is interrupted)
            interrupted -> stopAll(),
            // isFinished — never ends on its own; button release interrupts it
            () -> false,
            this
        );
    }

    public Command stopCommand() {
        return runOnce(this::stopAll);
    }

    // ---- Periodic ----

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/Flywheel/TargetRPM",            targetFlywheelRpm);
        Logger.recordOutput("Shooter/Flywheel/ActualRPM",            flywheelMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/Flywheel/AtSpeed",              flywheelAtSpeed());
        Logger.recordOutput("Shooter/Flywheel/CurrentAmps",          flywheelMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Flywheel/Faults",               flywheelMotor.getFaults().rawBits);
        Logger.recordOutput("Shooter/BackRoller/TargetRPM",          targetBackRollerRpm);
        Logger.recordOutput("Shooter/BackRoller/ActualRPM",          backRollerMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/BackRoller/AtSpeed",            backRollerAtSpeed());
        Logger.recordOutput("Shooter/BackRoller/CurrentAmps",        backRollerMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/BackRoller/Faults",             backRollerMotor.getFaults().rawBits);
        Logger.recordOutput("Shooter/Feed/TargetRPM",                targetFeedRpm);
        Logger.recordOutput("Shooter/Feed/ActualRPM",                feedMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/Feed/AtSpeed",                  feedAtSpeed());
        Logger.recordOutput("Shooter/Feed/CurrentAmps",              feedMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Feed/Faults",                   feedMotor.getFaults().rawBits);
        Logger.recordOutput("Shooter/FeedFollower/CurrentAmps",      feedFollowerMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/FeedFollower/Faults",           feedFollowerMotor.getFaults().rawBits);
        Logger.recordOutput("Shooter/Agitator/Output",               agitatorMotor.getAppliedOutput());
        Logger.recordOutput("Shooter/Agitator/Active",               agitatorActive);
        Logger.recordOutput("Shooter/Agitator/CurrentAmps",          agitatorMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Agitator/Faults",               agitatorMotor.getFaults().rawBits);
        Logger.recordOutput("Shooter/AgitatorFollower/CurrentAmps",  agitatorFollowerMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/AgitatorFollower/Faults",       agitatorFollowerMotor.getFaults().rawBits);
    }
}

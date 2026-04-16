package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.IntakeConstants;

public class IntakeRollerSubsystem extends SubsystemBase {

    private final SparkFlex rollerMotor;

    public IntakeRollerSubsystem() {
        rollerMotor = new SparkFlex(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.inverted(false);
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(IntakeConstants.ROLLER_CURRENT_LIMIT);
        config.signals
            .primaryEncoderPositionPeriodMs(500)
            .primaryEncoderVelocityPeriodMs(500)
            .appliedOutputPeriodMs(20)
            .motorTemperaturePeriodMs(500);
        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean isJammed() {
        return rollerMotor.getOutputCurrent() > IntakeConstants.ROLLER_JAM_CURRENT;
    }

    public void runIntake()     { rollerMotor.set(IntakeConstants.INTAKE_SPEED); }
    public void reverseIntake() { rollerMotor.set(IntakeConstants.OUTTAKE_SPEED); }
    public void stopIntake()    { rollerMotor.stopMotor(); }

    /** Run intake rollers inward while held; stop on release. */
    public Command intakeCommand() {
        return startEnd(this::runIntake, this::stopIntake);
    }

    /** Reverse intake rollers (eject) while held; stop on release. */
    public Command ejectCommand() {
        return startEnd(this::reverseIntake, this::stopIntake);
    }

    /** Stop intake rollers immediately (one-shot, for use in autos). */
    public Command stopCommand() {
        return runOnce(this::stopIntake);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake/Roller/Output", rollerMotor.getAppliedOutput());
    }
}

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

/**
 * A numeric constant that can be adjusted live from the dashboard when
 * {@link Constants#TUNING_MODE} is {@code true}.
 *
 * <p>In competition mode ({@code TUNING_MODE = false}) the value is fixed at the
 * default supplied at construction — there is zero runtime overhead and no
 * NetworkTables traffic.
 *
 * <p>In tuning mode the value is pushed to SmartDashboard under the given key
 * (visible in Shuffleboard, Glass, and AdvantageScope via NT4) and re-read from
 * the dashboard on every {@link #get()} call.
 *
 * <p>Usage:
 * <pre>
 *   private static final TunableNumber kAgitatorSpeed =
 *       new TunableNumber("Tuning/Shooter/AgitatorSpeed", ShooterConstants.AGITATOR_SPEED);
 *
 *   // inside a method called every loop:
 *   agitatorMotor.set(kAgitatorSpeed.get());
 * </pre>
 *
 * <p><b>Note:</b> values applied once at motor configuration (PID gains, ramp rates,
 * current limits) require a robot reboot to take effect regardless of tuning mode,
 * because they are written to the motor controller at construction time.
 */
public class TunableNumber {

    private final double defaultValue;
    private final String key;

    /**
     * @param key           SmartDashboard/NT key, e.g. {@code "Tuning/Shooter/AgitatorSpeed"}.
     *                      Only used when {@code TUNING_MODE} is {@code true}.
     * @param defaultValue  Fixed value in competition mode; initial dashboard value in tuning mode.
     */
    public TunableNumber(String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        if (Constants.TUNING_MODE) {
            SmartDashboard.putNumber(key, defaultValue);
        }
    }

    /**
     * Returns the current value.
     * In tuning mode this reads from SmartDashboard on every call; in competition mode
     * it returns the fixed default with no NT lookup.
     */
    public double get() {
        return Constants.TUNING_MODE ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
    }
}

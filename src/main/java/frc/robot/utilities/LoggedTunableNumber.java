package frc.robot.utilities;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import frc.robot.utilities.constants.Constants;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
*/

public class LoggedTunableNumber {
    private static final String tableKey = "TunableNumbers";

    private final String key;
    private boolean hasDefault = false;
    private double defaultValue;
    private LoggedDashboardNumber dashboardNumber;
    private double lastHasChangedValue;

    /**
     * Create a new LoggedTunableNumber
     * @param dashboardKey Key on dashboard
    */

    public LoggedTunableNumber(String dashboardKey) {
        this.key = tableKey + "/" + dashboardKey;
    }

    /**
     * Create a new LoggedTunableNumber with the default value
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
    */

    public LoggedTunableNumber(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     * @param defaultValue The default value
    */

    public void initDefault(double defaultValue) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;

            if (Constants.SwerveConstants.tuningMode) {
                dashboardNumber = new LoggedDashboardNumber(key, defaultValue);
            }
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     * @return The current value
    */

    public double get() {
        if (!hasDefault) {
            return 0.0;
        } else {
            return Constants.SwerveConstants.tuningMode ? dashboardNumber.get() : defaultValue;
        }
    }

    /**
     * Checks whether the number has changed since our last check
     * @return True if the number has changed since the last time this method was called, false otherwise
    */

    public boolean hasChanged() {
        double currentValue = get();
        if (currentValue != lastHasChangedValue) {
            lastHasChangedValue = currentValue;
            return true;
        }

        return false;
    }
}

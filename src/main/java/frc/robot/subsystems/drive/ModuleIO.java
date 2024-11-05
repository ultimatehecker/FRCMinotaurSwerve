package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionRadians = 0.0;
        public double driveVelocityRadiansPerSecond = 0.0;
        public double driveAppliedVoltage = 0.0;
        public double[] driveCurrentAmperes = new double[] {};
        public double[] driveTempuratureCelcius = new double[] {};

        public Rotation2d steeringAbsolutePosition = new Rotation2d();
        public Rotation2d steeringPosition = new Rotation2d();
        public double steeringVelocityRadiansPerSecond = 0.0;
        public double steeringAppliedVoltage = 0.0;
        public double[] steeringCurrentAmperes = new double[] {};
        public double[] steeringTempuratureCelcius = new double[] {};

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRadians = new double[] {};
        public Rotation2d[] odometrySteeringPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified voltage */
    public default void setDriveVoltage(double voltage) {}

    /** Run the steering motor at the specified voltage */
    public default void setSteeringVoltage(double voltage) {}

    /** Enable or disable brake mode on the drive motor */
    public default void setDriveBrakeMode(boolean brakeMode) {}

    /** Enable or disable brake mode on the steering motor */
    public default void setSteeringBrakeMode(boolean brakeMode) {}
}

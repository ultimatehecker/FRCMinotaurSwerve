package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
        public double yawVelocityRadiansPerSecond = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}
}

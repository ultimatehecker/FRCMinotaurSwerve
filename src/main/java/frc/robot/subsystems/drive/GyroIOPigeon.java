package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.constants.Constants;
import frc.robot.subsystems.drive.PhoenixOdometryThread;

import java.util.Queue;
import java.util.OptionalDouble;

public class GyroIOPigeon implements GyroIO {
    private final Pigeon2 gyro = new Pigeon2(Constants.SwerveConstants.gyroID);
    private final StatusSignal<Double> yaw = gyro.getYaw();
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<Double> yawVelocity = gyro.getAngularVelocityZWorld();

    public GyroIOPigeon(boolean pheonixDrive) {
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(Constants.SwerveConstants.odometryFrequency);
        yawVelocity.setUpdateFrequency(100);
        gyro.optimizeBusUtilization();

        if (pheonixDrive) {
            yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
            yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(gyro, gyro.getYaw());
        } else {
            yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
            yawPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(() -> {
                boolean valid = yaw.refresh().getStatus().isOK();
                if (valid) {
                    return OptionalDouble.of(yaw.getValueAsDouble());
                } else {
                    return OptionalDouble.empty();
                }
            });
        }
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadiansPerSecond = Units.degreesToRadians(yawVelocity.getValueAsDouble());

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}

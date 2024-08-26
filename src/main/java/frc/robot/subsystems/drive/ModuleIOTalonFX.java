package frc.robot.subsystems.drive;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.SwerveModuleConstants;

public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steeringMotor;
    private final CANcoder swerveEncoder;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> steeringPositionQueue;

    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVoltage;
    private final StatusSignal<Double> driveCurrent;

    private final StatusSignal<Double> swerveEncoderPosition;
    private final StatusSignal<Double> steeringPosition;
    private final StatusSignal<Double> steeringVelocity;
    private final StatusSignal<Double> steeringAppliedVoltage;
    private final StatusSignal<Double> steeringCurrent;

    private final Rotation2d swerveEncoderOffset;

    public ModuleIOTalonFX(SwerveModuleConstants constants) {
        driveMotor = new TalonFX(constants.driveMotorID);
        steeringMotor = new TalonFX(constants.steeringMotorID);
        swerveEncoder = new CANcoder(constants.swerveEnocderID);
        swerveEncoderOffset = constants.angleOffset;

        var driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 40.0;
        driveMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotor.getConfigurator().apply(driveMotorConfiguration);
        setDriveBrakeMode(true);

        var steeringMotorConfiguration = new TalonFXConfiguration();
        steeringMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 30.0;
        steeringMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        steeringMotor.getConfigurator().apply(steeringMotorConfiguration);
        setSteeringBrakeMode(true);

        swerveEncoder.getConfigurator().apply(new CANcoderConfiguration());

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveMotor, driveMotor.getPosition());
        steeringPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(steeringMotor, steeringMotor.getPosition());

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveAppliedVoltage = driveMotor.getMotorVoltage();
        driveCurrent = driveMotor.getStatorCurrent();

        swerveEncoderPosition = swerveEncoder.getPosition();
        steeringPosition = steeringMotor.getPosition();
        steeringVelocity = steeringMotor.getVelocity();
        steeringAppliedVoltage = steeringMotor.getMotorVoltage();
        steeringCurrent = steeringMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(Constants.SwerveConstants.odometryFrequency, drivePosition, steeringPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVoltage, driveCurrent, swerveEncoderPosition, steeringVelocity, steeringAppliedVoltage, steeringCurrent);

        driveMotor.optimizeBusUtilization();
        steeringMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVoltage, driveCurrent, swerveEncoderPosition, steeringPosition, steeringVelocity, steeringAppliedVoltage, steeringCurrent);

        inputs.drivePositionRadians = Units.rotationsToRadians(drivePosition.getValueAsDouble()) / Constants.SwerveConstants.ModuleGearing.getDriveReduction();
        inputs.driveVelocityRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(driveVelocity.getValueAsDouble()) / Constants.SwerveConstants.ModuleGearing.getDriveReduction();
        inputs.driveAppliedVoltage = driveAppliedVoltage.getValueAsDouble();
        inputs.driveCurrentAmperes = new double[] { driveCurrent.getValueAsDouble() };

        inputs.steeringAbsolutePositionRadians = Rotation2d.fromRotations(swerveEncoderPosition.getValueAsDouble()).minus(swerveEncoderOffset);
        inputs.steeringPositionRadians = Rotation2d.fromRotations(steeringPosition.getValueAsDouble() / Constants.SwerveConstants.ModuleGearing.getSteerReduction());
        inputs.steeringVelocityRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(steeringVelocity.getValueAsDouble() / Constants.SwerveConstants.ModuleGearing.getSteerReduction());
        inputs.steeringAppliedVoltage = steeringAppliedVoltage.getValueAsDouble();
        inputs.steeringCurrentAmperes = new double[] { steeringCurrent.getValueAsDouble() };

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRadians = drivePositionQueue.stream().mapToDouble((Double value) -> Units.rotationsToRadians(value) / Constants.SwerveConstants.ModuleGearing.getDriveReduction()).toArray();
        inputs.odometrySteeringPositions = steeringPositionQueue.stream().map((Double value) -> Rotation2d.fromRotations(value / Constants.SwerveConstants.ModuleGearing.getSteerReduction())).toArray(Rotation2d[]::new);

        timestampQueue.clear();
        drivePositionQueue.clear();
        steeringPositionQueue.clear();
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setSteeringVoltage(double voltage) {
        steeringMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        var configuration = new MotorOutputConfigs();
        configuration.Inverted = InvertedValue.CounterClockwise_Positive;
        configuration.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(configuration);
    }

    @Override
    public void setSteeringBrakeMode(boolean enable) {
        var configuration = new MotorOutputConfigs();
        configuration.Inverted = InvertedValue.CounterClockwise_Positive;
        configuration.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        steeringMotor.getConfigurator().apply(configuration);
    }
}

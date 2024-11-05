package frc.robot.subsystems.drive;

import java.util.OptionalDouble;
import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.constants.Constants.SwerveModuleConfiguration;
import frc.robot.utilities.constants.Constants;

public class ModuleIOSparkMax implements ModuleIO {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steeringMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steeringEncoder;
    private final CANcoder swerveEncoder;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> steeringPositionQueue;

    private final StatusSignal<Double> swerveEncoderPosition;

    private final Rotation2d swerveEncoderOffset;

    public ModuleIOSparkMax(SwerveModuleConfiguration moduleConfiguration) {
        driveMotor = new CANSparkMax(moduleConfiguration.driveMotorID(), MotorType.kBrushless);
        steeringMotor = new CANSparkMax(moduleConfiguration.steeringMotorID(), MotorType.kBrushless);
        swerveEncoder = new CANcoder(moduleConfiguration.swerveEnocderID());
        swerveEncoderOffset = moduleConfiguration.angleOffset();

        driveMotor.restoreFactoryDefaults();
        steeringMotor.restoreFactoryDefaults();

        driveMotor.setCANTimeout(250);
        steeringMotor.setCANTimeout(250);

        driveEncoder = driveMotor.getEncoder();
        steeringEncoder = steeringMotor.getEncoder();

        steeringMotor.setInverted(moduleConfiguration.isDriveInverted());
        driveMotor.setInverted(moduleConfiguration.isSteerInverted());
        driveMotor.setSmartCurrentLimit(Constants.ModuleConstants.driveSmartCurrentLimit);
        steeringMotor.setSmartCurrentLimit(Constants.ModuleConstants.steerSmartCurrentLimit);
        driveMotor.enableVoltageCompensation(Constants.ModuleConstants.moduleVoltageCompensation);
        steeringMotor.enableVoltageCompensation(Constants.ModuleConstants.moduleVoltageCompensation);

        driveEncoder.setPosition(0.0);
        driveEncoder.setMeasurementPeriod(10);
        driveEncoder.setAverageDepth(2);

        steeringEncoder.setPosition(0.0);
        steeringEncoder.setMeasurementPeriod(10);
        steeringEncoder.setAverageDepth(2);

        swerveEncoder.getConfigurator().apply(new CANcoderConfiguration());

        driveMotor.setCANTimeout(0);
        steeringMotor.setCANTimeout(0);

        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int)(1000.0 / Constants.SwerveConstants.odometryFrequency));
        steeringMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int)(1000.0 / Constants.SwerveConstants.odometryFrequency));
        timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(() -> {
            double value = driveEncoder.getPosition();
            if(driveMotor.getLastError() == REVLibError.kOk) {
                return OptionalDouble.of(value);   
            } else {
                return OptionalDouble.empty();
            }
        });

        steeringPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(() -> {
            double value = steeringEncoder.getPosition();
            if(steeringMotor.getLastError() == REVLibError.kOk) {
                return OptionalDouble.of(value);   
            } else {
                return OptionalDouble.empty();
            }
        });

        swerveEncoderPosition = swerveEncoder.getAbsolutePosition();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, swerveEncoderPosition);

        driveMotor.burnFlash();
        steeringMotor.burnFlash();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(swerveEncoderPosition);

        inputs.drivePositionRadians = Units.rotationsToRadians(driveEncoder.getPosition()) / Constants.SwerveConstants.moduleGearing.getDriveReduction();
        inputs.driveVelocityRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / Constants.SwerveConstants.moduleGearing.getDriveReduction();
        inputs.driveAppliedVoltage = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrentAmperes = new double[] { driveMotor.getOutputCurrent() };
        inputs.driveTempuratureCelcius = new double[] { driveMotor.getMotorTemperature() };

        inputs.steeringAbsolutePosition = new Rotation2d(swerveEncoderPosition.getValueAsDouble()).minus(swerveEncoderOffset);
        inputs.steeringPosition = Rotation2d.fromRotations(steeringEncoder.getPosition() / Constants.SwerveConstants.moduleGearing.getSteerReduction());
        inputs.steeringVelocityRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(steeringEncoder.getVelocity() / Constants.SwerveConstants.moduleGearing.getSteerReduction());
        inputs.steeringAppliedVoltage = steeringMotor.getAppliedOutput() * steeringMotor.getBusVoltage();
        inputs.steeringCurrentAmperes = new double[] { steeringMotor.getOutputCurrent() };
        inputs.steeringTempuratureCelcius = new double[] { steeringMotor.getMotorTemperature() };

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRadians = drivePositionQueue.stream().mapToDouble((Double value) -> Units.rotationsToRadians(value) / Constants.SwerveConstants.moduleGearing.getDriveReduction()).toArray();
        inputs.odometrySteeringPositions = steeringPositionQueue.stream().map((Double value) -> Rotation2d.fromRotations(value / Constants.SwerveConstants.moduleGearing.getSteerReduction())).toArray(Rotation2d[]::new);

        timestampQueue.clear();
        drivePositionQueue.clear();
        steeringPositionQueue.clear();
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveMotor.set(voltage);
    }

    @Override
    public void setSteeringVoltage(double voltage) {
        steeringMotor.set(voltage);
    }

    @Override
    public void setDriveBrakeMode(boolean brakeMode) {
        driveMotor.setIdleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setSteeringBrakeMode(boolean brakeMode) {
        steeringMotor.setIdleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    }
}

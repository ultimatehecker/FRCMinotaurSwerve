package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utilities.constants.Constants;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController drivePIDController;
    private final PIDController steeringPIDController;

    private Rotation2d angleSetpoint = null;
    private Double speedSetpoint = null;
    private Rotation2d steeringPositionOffset = null;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        switch(Constants.getMode()) {
            case REAL:
            case REPLAY:
                driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
                drivePIDController = new PIDController(0.05, 0.0, 0.0);
                steeringPIDController = new PIDController(7.0, 0.0, 0.0);
                break;
            case SIM:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
                drivePIDController = new PIDController(0.1, 0.0, 0.0);
                steeringPIDController = new PIDController(10.0, 0.0, 0.0);
                break;
            default:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
                drivePIDController = new PIDController(0.0, 0.0, 0.0);
                steeringPIDController = new PIDController(0.0, 0.0, 0.0);
                break;
        }

        steeringPIDController.enableContinuousInput(-Math.PI, Math.PI);
        setBrakeMode(true);
    }

    public void updateInputs() {
        io.updateInputs(inputs);
    }

    public void periodic() {
        Logger.processInputs("Drivetrain/Module" + Integer.toString(index), inputs);

        if (steeringPositionOffset == null && inputs.steeringAbsolutePosition.getRadians() != 0.0) {
            steeringPositionOffset = inputs.steeringAbsolutePosition.minus(inputs.steeringPosition);
        }

        if(angleSetpoint != null) {
            io.setSteeringVoltage(steeringPIDController.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

            if(speedSetpoint != null) {
                double adjustmentSpeedSetpoint = speedSetpoint  * Math.cos(steeringPIDController.getPositionError());
                double velocityRadiansPerSecond = adjustmentSpeedSetpoint / (Constants.SwerveConstants.drivetrainConfiguration.wheelRadius());
                io.setDriveVoltage(driveFeedforward.calculate(velocityRadiansPerSecond) + drivePIDController.calculate(inputs.driveVelocityRadiansPerSecond, velocityRadiansPerSecond));
            }
        }

        int sampleCount = inputs.odometryTimestamps.length;
        odometryPositions = new SwerveModulePosition[sampleCount];

        for(int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRadians[i] * (Constants.SwerveConstants.drivetrainConfiguration.wheelRadius());
            Rotation2d angle = inputs.odometrySteeringPositions[i].plus(steeringPositionOffset != null ? steeringPositionOffset : new Rotation2d());
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        var optimizedState = SwerveModuleState.optimize(state, getAngle());
        angleSetpoint = optimizedState.angle;
        speedSetpoint = optimizedState.speedMetersPerSecond;
        return optimizedState;
    }

    public void runCharacterization(double voltage) {
        angleSetpoint = new Rotation2d();
        io.setDriveVoltage(voltage);
        speedSetpoint = null;
    }

    public void stop() {
        io.setDriveVoltage(0.0);
        io.setSteeringVoltage(0.0);

        angleSetpoint = null;
        speedSetpoint = null;
    }

    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setSteeringBrakeMode(enabled);
    }

    public Rotation2d getAngle() {
        if(steeringPositionOffset == null) {
            return new Rotation2d();
        } else {
            return inputs.steeringPosition.plus(steeringPositionOffset);
        }
    }

    public double getPositionMeters() {
        return inputs.drivePositionRadians * (Constants.SwerveConstants.drivetrainConfiguration.wheelRadius());
    }

    public double getVelocityMetersPerSecond() {
        return inputs.driveVelocityRadiansPerSecond * (Constants.SwerveConstants.drivetrainConfiguration.wheelRadius());
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getVelocityMetersPerSecond(), getAngle());
    }

    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    public double getCharacterizationVelocity() {
        return inputs.driveVelocityRadiansPerSecond;
    }
}

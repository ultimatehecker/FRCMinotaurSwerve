package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utilities.SwerveModuleGearing;
import frc.robot.utilities.constants.Constants;

public class ModuleIOSim implements ModuleIO {
    private static final double LOOP_PERIOD_SECONDS = 0.02;
    private static final SwerveModuleGearing ModuleGearing = Constants.SwerveConstants.ModuleGearing;

    private DCMotorSim driveMotor = new DCMotorSim(DCMotor.getNEO(1), ModuleGearing.getDriveReduction(), 0.025);
    private DCMotorSim steeringMotor = new DCMotorSim(DCMotor.getNEO(1), ModuleGearing.getSteerReduction(), 0.004);

    private final Rotation2d steeringAbsolutePosition = new Rotation2d(Math.random() * 2 * Math.PI);
    private double driveAppliedVoltage = 0.0;
    private double steeringAppliedVoltage = 0.0;

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveMotor.update(LOOP_PERIOD_SECONDS);
        steeringMotor.update(LOOP_PERIOD_SECONDS);

        inputs.drivePositionRadians = driveMotor.getAngularPositionRad();
        inputs.driveVelocityRadiansPerSecond = driveMotor.getAngularVelocityRadPerSec();
        inputs.driveAppliedVoltage = driveAppliedVoltage;
        inputs.driveCurrentAmperes = new double [] { Math.abs(driveMotor.getCurrentDrawAmps()) };

        inputs.steeringAbsolutePositionRadians = new Rotation2d(steeringMotor.getAngularPositionRad()).plus(steeringAbsolutePosition);
        inputs.steeringPositionRadians = new Rotation2d(steeringMotor.getAngularPositionRad());
        inputs.steeringVelocityRadiansPerSecond = steeringMotor.getAngularVelocityRadPerSec();
        inputs.steeringAppliedVoltage = steeringAppliedVoltage;
        inputs.steeringCurrentAmperes = new double [] { Math.abs(steeringMotor.getCurrentDrawAmps()) };

        inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
        inputs.odometryDrivePositionsRadians = new double[] { inputs.drivePositionRadians };
        inputs.odometrySteeringPositions = new Rotation2d[] { inputs.steeringPositionRadians };
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveAppliedVoltage = MathUtil.clamp(voltage, -12, 12);
        driveMotor.setInputVoltage(voltage);
    }

    @Override
    public void setSteeringVoltage(double voltage) {
        steeringAppliedVoltage = MathUtil.clamp(voltage, -12, 12);
        driveMotor.setInputVoltage(voltage);
    }
}

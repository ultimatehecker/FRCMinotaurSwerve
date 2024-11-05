package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;

import frc.robot.utilities.VisionHelpers.TimestampedVisionUpdate;
import frc.robot.utilities.constants.Constants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class SwerveSubsystem extends SubsystemBase {
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroIOInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4];
    private final SysIdRoutine sysID;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.SwerveConstants.moduleTranslations);
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        rawGyroRotation,
        lastModulePositions,
        new Pose2d(),
        Constants.VisionConstants.stateStdDevs,
        new Matrix<>(VecBuilder.fill(Constants.VisionConstants.xyStdDevCoefficient, Constants.VisionConstants.xyStdDevCoefficient, Constants.VisionConstants.thetaStdDevCoefficient))
    );

    private SwerveDrivePoseEstimator odometryDrive = new SwerveDrivePoseEstimator(
        kinematics, 
        rawGyroRotation, 
        lastModulePositions,
        new Pose2d()
    );

    public SwerveSubsystem(GyroIO gyroIO, ModuleIO flModule, ModuleIO frModule, ModuleIO blModule, ModuleIO brModule) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModule, 0);
        modules[1] = new Module(frModule, 1);
        modules[2] = new Module(blModule, 2);
        modules[3] = new Module(brModule, 3);

        PhoenixOdometryThread.getInstance().start();
        SparkMaxOdometryThread.getInstance().start();

        AutoBuilder.configureHolonomic(
            null, 
            null, 
            () -> kinematics.toChassisSpeeds(), 
            null, 
            null, 
            null, 
            null
        );

        sysID =new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this
            )
        );
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        gyroIO.updateInputs(gyroIOInputs);

        for(var module : modules) {
            module.updateInputs();
        }

        odometryLock.unlock();
        Logger.processInputs("Drivetrain/Gyroscope", gyroIOInputs);

        for(var module : modules) {
            module.periodic();
        }

        if(DriverStation.isDisabled()) {
            for(var module : modules) {
                module.stop();
            }
        }

        if(DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {}); 
        }

        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;

        for(int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

            for(int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters, modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            if(gyroIOInputs.connected) {
                rawGyroRotation = gyroIOInputs.odometryYawPositions[i];
            } else {
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
            odometryDrive.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }
    }

    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discretSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discretSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.SwerveConstants.drivetrainConfiguration.maxLinearVelocity());

        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++) {
            optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
        }

        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointOptimized", optimizedSetpointStates);
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for(int i = 0; i < 4; i++) {
            headings[i] = Constants.SwerveConstants.moduleTranslations[i].getAngle();
        }

        kinematics.resetHeadings(headings);
        stop();
    }

    public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
        return sysID.quasistatic(direction);
    }

    public Command sysIDDynamic(SysIdRoutine.Direction direction) {
        return sysID.dynamic(direction);
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++) {
            states[i] = modules[i].getSwerveModuleState();
        }

        return states;
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++) {
            states[i] = modules[i].getSwerveModulePosition();
        }
        
        return states;
    }

    @AutoLogOutput(key = "Odometry/PoseEstimation")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "Odometry/Drive")
    public Pose2d getDrivePose() {
        return odometryDrive.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getSwerveModulePositions(), pose);
    }

    public void setAutoStartPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getSwerveModulePositions(), pose);
        odometryDrive.resetPosition(rawGyroRotation, getSwerveModulePositions(), pose);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
    }

    public void addVisionData(List<TimestampedVisionUpdate> visionData) {
        visionData.forEach(visionUpdate -> {
            addVisionMeasurement(visionUpdate.pose(), visionUpdate.timestamp(), visionUpdate.stdDevs());
        });
    }
}

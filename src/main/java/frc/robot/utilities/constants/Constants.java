package frc.robot.utilities.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.utilities.Alert;
import frc.robot.utilities.Alert.AlertType;
import frc.robot.utilities.constants.Constants.SwerveModuleConfiguration;
import frc.robot.utilities.constants.Constants.SwerveModuleParameters;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
*/

public class Constants {
    public static enum RobotType { MINOBOT, SIMBOT };
    public static enum Mode { REAL, REPLAY, SIM };
    public static final double loopPeriodSecs = 0.02;
    public static final double loopPeriodMs = loopPeriodSecs * 1000.0;

    private static final Alert invalidRobotAlert = new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);
    private static final RobotType robot = RobotType.MINOBOT;
    public record HeadingControllerConstants(double Kp, double Kd) {}

    public static RobotType getRobot() {
        if (RobotBase.isReal()) {
            if (robot == RobotType.SIMBOT) { // Invalid robot selected
                invalidRobotAlert.set(true);
                return RobotType.MINOBOT;
            } else {
                return robot;
            }
        } else {
            return robot;
        }
    }

    public static Mode getMode() {
        switch (getRobot()) {
            case MINOBOT:
                return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            case SIMBOT:
                return Mode.SIM;
            default:
                return Mode.REAL;
        }
    }

    // public static final Map<RobotType, String> logFolders = Map.of(RobotType.MINOBOT, "/media/sda2");

    public static final class ModuleConstants {
        public static SwerveModuleConfiguration[] swerveModuleConfiguration = switch (Constants.getRobot()) {
            case MINOBOT -> new SwerveModuleConfiguration[] {
                new SwerveModuleConfiguration(1, 2, 9, Rotation2d.fromRotations(1), false, false),
                new SwerveModuleConfiguration(3, 4, 10, Rotation2d.fromRotations(1), false, false),
                new SwerveModuleConfiguration(5, 6, 11, Rotation2d.fromRotations(1), false, false),
                new SwerveModuleConfiguration(7, 8, 12, Rotation2d.fromRotations(1), false, false)
            };
            case SIMBOT -> new SwerveModuleConfiguration[] {
                new SwerveModuleConfiguration(0, 0, 0, new Rotation2d(0), false, false),
                new SwerveModuleConfiguration(0, 0, 0, new Rotation2d(0), false, false),
                new SwerveModuleConfiguration(0, 0, 0, new Rotation2d(0), false, false),
                new SwerveModuleConfiguration(0, 0, 0, new Rotation2d(0), false, false)
            };
        };

        public static SwerveModuleParameters swerveModuleParameters = switch(Constants.getRobot()) {
            case MINOBOT -> new SwerveModuleParameters(
                0.1,
                0.13,
                0.1,
                0.0,
                10.0,
                0.0,
                SwerveModuleGearing.MK3_FAST.driveReduction,
                SwerveModuleGearing.MK3_FAST.steerReduction
            );
            case SIMBOT -> new SwerveModuleParameters(
                0.014,
                0.134,
                0.1,
                0.0,
                10.0,
                0.0, 
                SwerveModuleGearing.MK3_FAST.driveReduction,
                SwerveModuleGearing.MK3_FAST.steerReduction
            );
        };

        public static final int driveSmartCurrentLimit = 40;
        public static final int steerSmartCurrentLimit = 30;
        public static final double moduleVoltageCompensation = 12.0;
    }

    public static final class SwerveConstants {
        public static DrivetrainConfiguration drivetrainConfiguration = switch(Constants.getRobot()) {
            default -> new DrivetrainConfiguration(
                Units.inchesToMeters(2.0), 
                Units.inchesToMeters(28.0), 
                Units.inchesToMeters(28.0), 
                Units.feetToMeters(12.16), 
                Units.feetToMeters(12.16), 
                Units.feetToMeters(12.16), 
                Units.feetToMeters(12.16)
            );
        };

        public static HeadingControllerParameters headingControllerParameters = switch(Constants.getRobot()) {
            case MINOBOT -> new HeadingControllerParameters(3.0, 0.0);
            case SIMBOT -> new HeadingControllerParameters(3.0, 0.0);
        };

        public static final Translation2d[] moduleTranslations =
            new Translation2d[] {
                new Translation2d(drivetrainConfiguration.trackwidthX() / 2.0, drivetrainConfiguration.trackwidthY() / 2.0),
                new Translation2d(drivetrainConfiguration.trackwidthX() / 2.0, -drivetrainConfiguration.trackwidthY() / 2.0),
                new Translation2d(-drivetrainConfiguration.trackwidthX() / 2.0, drivetrainConfiguration.trackwidthY() / 2.0),
                new Translation2d(-drivetrainConfiguration.trackwidthX() / 2.0, -drivetrainConfiguration.trackwidthY() / 2.0)
        };

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
        public static final double odometryFrequency = switch(Constants.getRobot()) {
            case SIMBOT -> 50.0;
            case MINOBOT -> 250.0;
        };
        public static final SwerveModuleGearing moduleGearing = SwerveModuleGearing.MK3_FAST;

        public static final boolean tuningMode = true;
        public static final int gyroID = 13;
    }

    public static final class VisionConstants {
        public static final Matrix<N3, N1> stateStdDevs = switch (Constants.getRobot()) { 
            default -> new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
        };

        public static final double xyStdDevCoefficient = switch (Constants.getRobot()) {
            default -> 0.01;
        };

        public static final double thetaStdDevCoefficient = switch (Constants.getRobot()) {
            default -> 0.01;
        };
    }

    public enum SwerveModuleGearing {
        MK3_STANDARD(SDSModuleGearRatios.MK3_STANDARD, false, SDSModuleGearRatios.MK3_STEERING, false),
        MK3_FAST(SDSModuleGearRatios.MK3_FAST, false, SDSModuleGearRatios.MK3_STEERING, false),
        MK4_L1(SDSModuleGearRatios.MK4_L1, false, SDSModuleGearRatios.MK4_STEERING, false),
        MK4_L2(SDSModuleGearRatios.MK4_L2, false, SDSModuleGearRatios.MK4_STEERING, false),
        MK4_L3(SDSModuleGearRatios.MK4_L3, false, SDSModuleGearRatios.MK4_STEERING, false),
        MK4_L4(SDSModuleGearRatios.MK4_L4, false, SDSModuleGearRatios.MK4_STEERING, false),
        MK4I_L1(SDSModuleGearRatios.MK4I_L1, false, SDSModuleGearRatios.MK4I_STEERING, true),
        MK4I_L2(SDSModuleGearRatios.MK4I_L2, false, SDSModuleGearRatios.MK4I_STEERING, true),
        MK4I_L3(SDSModuleGearRatios.MK4I_L3, false, SDSModuleGearRatios.MK4I_STEERING, true),
        MK4N_L1(SDSModuleGearRatios.MK4N_L1, false, SDSModuleGearRatios.MK4N_STEERING, true),
        MK4N_L2(SDSModuleGearRatios.MK4N_L2, false, SDSModuleGearRatios.MK4N_STEERING, true),
        MK4N_L3(SDSModuleGearRatios.MK4N_L3, false, SDSModuleGearRatios.MK4N_STEERING, true);
    
        private double driveReduction;
        private boolean driveInverted;
        private double steerReduction;
        private boolean steerInverted;
    
        /**
         * Creates a new COTS Module Gearing Configuration.
         * @param driveReduction The overall drive reduction of the module. Multiplying motor rotations by this value should result in wheel rotations.
         * @param driveInverted  Whether the drive motor should be inverted. If there is an odd number of gea reductions this is typically true.
         * @param steerReduction The overall steer reduction of the module. Multiplying motor rotations by this value should result in rotations of the steering pulley.
         * @param steerInverted  Whether the steer motor should be inverted. If there is an odd number of gear reductions this is typically true.
         */
    
        SwerveModuleGearing(double driveReduction, boolean driveInverted, double steerReduction, boolean steerInverted) {
            this.driveReduction = driveReduction;
            this.driveInverted = driveInverted;
            this.steerReduction = steerReduction;
            this.steerInverted = steerInverted;
        }
    
        /**
         * Gets the overall reduction of the drive system. If this value is multiplied by drive motor rotations the result would be drive wheel rotations.
         */
    
        public double getDriveReduction() {
            return driveReduction;
        }
    
        /**
         * Gets if the drive motor should be inverted.
         */
    
        public boolean isDriveInverted() {
            return driveInverted;
        }
    
        /**
         * Gets the overall reduction of the steer system. If this value is multiplied by steering motor rotations the result would be steering pulley rotations.
         */
    
        public double getSteerReduction() {
            return steerReduction;
        }
    
        /**
         * Gets if the steering motor should be inverted.
         */
    
        public boolean isSteerInverted() {
            return steerInverted;
        }
    
        /**
         * Swerve Module Gearing.
         * A collection of every single gear ratio for every COTS swerve module.
         */
    
         private class SDSModuleGearRatios {
            public static final double MK3_STANDARD = (50.0 / 14.0) * (16.0 / 28.0) * (60.0 / 15.0);
            public static final double MK3_FAST = (48.0 / 16.0) * (16.0 / 28.0) * (60.0 / 15.0);
    
            public static final double MK4_L1 = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);
            public static final double MK4_L2 = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
            public static final double MK4_L3 = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
            public static final double MK4_L4 = (48.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    
            public static final double MK4I_L1 = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);
            public static final double MK4I_L2 = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
            public static final double MK4I_L3 = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    
            public static final double MK4N_L1 = (50.0 / 16.0) * (19.0 / 25.0) * (45.0 / 15.0);
            public static final double MK4N_L2 = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
            public static final double MK4N_L3 = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    
            public static final double MK3_STEERING = (32.0 / 15.0) * (60.0 / 10.0);
            public static final double MK4_STEERING = (32.0 / 15.0) * (60.0 / 10.0);
            public static final double MK4I_STEERING = (50.0 / 14.0) * (60.0 / 10.0);
            public static final double MK4N_STEERING = 18.75;
        }
    }

    public final class BuildConstants {
        public static final String MAVEN_GROUP = "";
        public static final String MAVEN_NAME = "B-Swerve-Copy";
        public static final String VERSION = "unspecified";
        public static final int GIT_REVISION = -1;
        public static final String GIT_SHA = "UNKNOWN";
        public static final String GIT_DATE = "UNKNOWN";
        public static final String GIT_BRANCH = "UNKNOWN";
        public static final String BUILD_DATE = "2023-10-20 09:11:27 EDT";
        public static final long BUILD_UNIX_TIME = 1697807487606L;
        public static final int DIRTY = 129;
      
        private BuildConstants(){}
    }

    public record SwerveModuleConfiguration(
        int driveMotorID, 
        int steeringMotorID, 
        int swerveEnocderID, 
        Rotation2d angleOffset, 
        boolean isSteerInverted,
        boolean isDriveInverted
    ) {}

    public record SwerveModuleParameters(
      double ffKs,
      double ffKv,
      double driveKp,
      double drivekD,
      double turnKp,
      double turnkD,
      double driveReduction,
      double turnReduction) {}

    public record DrivetrainConfiguration(
        double wheelRadius, 
        double trackWidthX, 
        double trackWidthY, 
        double maxLinearVelocity, 
        double maxLinearAcceleration,
        double maxAngularVelocity,
        double maxAngularAcceleration
    ) {
        public double driveBaseRadius() {
            return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
        }

        public double trackwidthX() {
            return trackWidthX;
        }

        public double trackwidthY() {
            return trackWidthY;
        }
    }

    public record HeadingControllerParameters(
        double Kp, 
        double Kd
    ) {}
}

package frc.robot.utilities.constants;

import com.google.flatbuffers.FlexBuffers.Map;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utilities.Alert;
import frc.robot.utilities.Alert.AlertType;
import frc.robot.utilities.SwerveModuleGearing;

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

    private static final Alert invalidRobotAlert = new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);
    private static final RobotType robot = RobotType.MINOBOT;

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

    public static final class ModuleConstants {}
    public static final class SwerveConstants {
        public static boolean tuningMode = false;
        public static final SwerveModuleGearing ModuleGearing = SwerveModuleGearing.MK3_FAST;
        public static final double odometryFrequency = switch (Constants.getRobot()) {
            case SIMBOT -> 50.0;
            case MINOBOT -> 250.0;
        };
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
}

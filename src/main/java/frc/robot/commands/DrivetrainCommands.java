package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.drive.SwerveStateMachine;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.utilities.constants.Constants;

public class DrivetrainCommands {
    private static final double controllerDeadband = 0.1;

    private DrivetrainCommands() {}

    public static final Command teleopDrive(SwerveSubsystem swerveSubsystem, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        ProfiledPIDController headingController = new ProfiledPIDController(
            Constants.SwerveConstants.headingControllerParameters.Kp(),
            0,
            Constants.SwerveConstants.headingControllerParameters.Kd(),
            new TrapezoidProfile.Constraints(Constants.SwerveConstants.drivetrainConfiguration.maxAngularVelocity(), Constants.SwerveConstants.drivetrainConfiguration.maxAngularAcceleration()),
            Constants.loopPeriodMs
        );

        headingController.reset(swerveSubsystem.getRotation().getRadians());
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.run(() -> {
           double linearMagnitude = MathUtil.applyDeadband(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), controllerDeadband); 
           Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
           double omega = MathUtil.applyDeadband(thetaSupplier.getAsDouble(), controllerDeadband);

           linearMagnitude = linearMagnitude * linearMagnitude;
           omega = Math.copySign(omega * omega, omega);

            if(SwerveStateMachine.getInstance().isHeadingControlled()) {
                linearMagnitude = Math.min(linearMagnitude, 0.75);
            }

            Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();
            boolean isFlipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

            Optional<Rotation2d> targetGyroAngle = Optional.empty();
            Rotation2d measuredGyroAngle = swerveSubsystem.getRotation();
            double feedForwardRadialVelocity = 0.0;

            double robotRelativeXVelocity = linearVelocity.getX() * Constants.SwerveConstants.drivetrainConfiguration.maxLinearVelocity();
            double robotRelativeYVelocity = linearVelocity.getY() * Constants.SwerveConstants.drivetrainConfiguration.maxLinearVelocity();

            if(SwerveStateMachine.getInstance().isHeadingControlled()) {
                measuredGyroAngle = swerveSubsystem.getPose().getRotation();
                SwerveStateMachine.getInstance().calculateHeading(swerveSubsystem.getPose());
                targetGyroAngle = Optional.of(SwerveStateMachine.getInstance().getHeadingAngle());
            }

            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                robotRelativeXVelocity, 
                robotRelativeYVelocity,
                SwerveStateMachine.getInstance().isHeadingControlled() && targetGyroAngle.isPresent() ? feedForwardRadialVelocity + headingController.calculate(measuredGyroAngle.getRadians(), targetGyroAngle.get().getRadians()) : omega * Constants.SwerveConstants.drivetrainConfiguration.maxAngularVelocity(),
                isFlipped ? swerveSubsystem.getRotation().plus(new Rotation2d(Math.PI)) : swerveSubsystem.getRotation()
            );

            swerveSubsystem.runVelocity(chassisSpeeds);
        }, swerveSubsystem);
    }
}
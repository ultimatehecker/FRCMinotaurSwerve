// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.commands.DrivetrainCommands;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.drive.SwerveStateMachine;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.Constants.ModuleConstants.*;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem;

  private final CommandXboxController DriverController = new CommandXboxController(0);
  private final CommandXboxController OperatorController = new CommandXboxController(1);

  private final LoggedDashboardChooser<Command> autonomousChooser;

  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL:
        swerveSubsystem = new SwerveSubsystem(new GyroIONavX(), new ModuleIOSparkMax(FrontLeftModule.constants), new ModuleIOSparkMax(FrontRightModule.constants), new ModuleIOSparkMax(BackLeftModule.constants), new ModuleIOSparkMax(BackRightModule.constants));
        break;

      case SIM:
        swerveSubsystem = new SwerveSubsystem(new GyroIONavX(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
        break;
      default:
        swerveSubsystem = new SwerveSubsystem(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        break;
    }

    autonomousChooser = new LoggedDashboardChooser<>("Autonomous Chooser", AutoBuilder.buildAutoChooser());
    autonomousChooser.addOption("Drivetrain SysID (Quasistatic Forward)", swerveSubsystem.sysIDQuasistatic(SysIdRoutine.Direction.kForward));
    autonomousChooser.addOption("Drivetrain SysID (Quasistatic Reverse)", swerveSubsystem.sysIDQuasistatic(SysIdRoutine.Direction.kReverse));
    autonomousChooser.addOption("Drivetrain SysID (Dynamic Forward)", swerveSubsystem.sysIDDynamic(SysIdRoutine.Direction.kForward));
    autonomousChooser.addOption("Drivetrain SysID (Dynamic Reverse)", swerveSubsystem.sysIDDynamic(SysIdRoutine.Direction.kReverse));

    SwerveStateMachine.getInstance().disableHeadingControl();
    configureDriverController();
    configureOperatorController();
  }

  public void configureDriverController() {
    swerveSubsystem.setDefaultCommand(DrivetrainCommands.teleopDrive(
      swerveSubsystem, 
      () -> -DriverController.getLeftY(), 
      () -> -DriverController.getLeftX(), 
      () -> -DriverController.getRightX()
    ));

    DriverController.leftBumper().whileTrue(Commands.runOnce(() -> SwerveStateMachine.getInstance().toggleDriveMode()));
  }

  public void configureOperatorController() {}

  public void onDisabled() {}
  public void onEnabled() {}

  public Command getAutonomousCommand() {
    return Commands.print("No Autonomous Commands have been set up");
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  public RobotContainer() {}

  public void configureDriveController() {}
  public void configureOperatorController() {}

  public void onDisabled() {}
  public void onEnabled() {}

  public Command getAutonomousCommand() {
    return Commands.print("No Autonomous Commands have been set up");
  }
}

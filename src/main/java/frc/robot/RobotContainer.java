// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsytems.DriveSubsystem;

import static frc.robot.Constants.*;

public class RobotContainer {
  private final DriveSubsystem driveSubsys = new DriveSubsystem();

  private final CommandXboxController xboxController = new CommandXboxController(DriveConstants.xboxControllerPort);

  public RobotContainer() {
    configureBindings();
    configDefaultCommands();
  }

  private void configDefaultCommands() {
    driveSubsys.setDefaultCommand(driveSubsys.arcadeDrive(() -> xboxController.getLeftY(),() -> -xboxController.getRightX()));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

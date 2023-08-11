/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsytems.CannonSubsystem;
import frc.robot.subsytems.DriveSubsystem;
import static frc.robot.Constants.*;

public class RobotContainer {
  private final DriveSubsystem driveSubsys = new DriveSubsystem();

  private final CommandXboxController xboxController = new CommandXboxController(DriveConstants.xboxControllerPort);

  public final CannonSubsystem cannonSubsys = new CannonSubsystem(xboxController);

  public RobotContainer() {
    configBindings();
    configDefaultCommands();
  }

  private void configDefaultCommands() {
    driveSubsys
        .setDefaultCommand(driveSubsys.arcadeDrive(() -> xboxController.getLeftY(), () -> -xboxController.getRightX()));
    cannonSubsys.setDefaultCommand(cannonSubsys.holdAngle());
  }

  private void configBindings() {
    xboxController.rightTrigger().onTrue(cannonSubsys.fireCannon());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

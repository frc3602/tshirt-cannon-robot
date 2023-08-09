/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.robot.subsytems;

import frc.robot.subsytems.CannonSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControllerSubsystem extends SubsystemBase {

  CannonSubsystem cannonSubsys;
  public ControllerSubsystem(CannonSubsystem cannonSubsystem) {
   cannonSubsys = cannonSubsystem;
   cannonSubsys.cannonCommands.addCommands(cannonSubsys.holdAngle());
  }

  public Command addFireCannon() {
    return runOnce(() -> {
      cannonSubsys.cannonCommands.addCommands(cannonSubsys.fireCannon());
    });
  }
}

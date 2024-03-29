/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.robot.subsytems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax frontLeftMotor = new CANSparkMax(DriveConstants.frontLeftMotorCANID, MotorType.kBrushless);
  private final CANSparkMax backLeftMotor = new CANSparkMax(DriveConstants.backLeftMotorCANID, MotorType.kBrushless);
  private final CANSparkMax frontRightMotor = new CANSparkMax(DriveConstants.frontRightMotorCANID, MotorType.kBrushless);
  private final CANSparkMax backRightMotor = new CANSparkMax(DriveConstants.backRightMotorCANID, MotorType.kBrushless);

  private final MotorControllerGroup leftControllerGroup = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
  private final MotorControllerGroup rightControllerGroup = new MotorControllerGroup(frontRightMotor, backRightMotor);

  private final DifferentialDrive diffDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

  public DriveSubsystem() {
    configDriveSubsys();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command arcadeDrive(DoubleSupplier speed, DoubleSupplier rotation) {
    return run(() -> {
      diffDrive.arcadeDrive(speed.getAsDouble(), rotation.getAsDouble());
    }).withName("Arcade Drive Command");
  }

  private void configDriveSubsys() {
    rightControllerGroup.setInverted(true);

    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    backLeftMotor.setIdleMode(IdleMode.kBrake);
    frontRightMotor.setIdleMode(IdleMode.kBrake);
    backRightMotor.setIdleMode(IdleMode.kBrake);
  }
}

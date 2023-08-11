/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.robot.subsytems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CannonSubsystem extends SubsystemBase {
  private CommandXboxController xBoxController;

  // Motor controllers
  private final CANSparkMax rotateMotor = new CANSparkMax(CannonConstants.rotateMotorCANID, MotorType.kBrushless);

  // PID controllers
  private final SparkMaxPIDController rotateMotorPIDController = rotateMotor.getPIDController();

  // Motor encoders
  private final RelativeEncoder rotateMotorEncoder = rotateMotor.getEncoder();

  // Pneumatics
  private final PneumaticHub pneumaticHub = new PneumaticHub(CannonConstants.pneumaticHubCANID);
  private final DoubleSolenoid firesSolenoid = new DoubleSolenoid(CannonConstants.pneumaticHubCANID,
      PneumaticsModuleType.REVPH, CannonConstants.fillSolenoidChan, CannonConstants.firesSolenoidChan);

  public final DoubleSolenoid loadActuatorSolenoid = new DoubleSolenoid(CannonConstants.pneumaticHubCANID,
      PneumaticsModuleType.REVPH, CannonConstants.loadActuatorSolenoidFwdChan,
      CannonConstants.loadActuatorSolenoidRevChan);

  // DIO
  private final DigitalInput loadActuatorSwitch = new DigitalInput(CannonConstants.loadActuatorSwitchDIOChan);
  public final SparkMaxLimitSwitch rotatePresetLimit = rotateMotor
      .getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  public double rotationAngleDegrees;
  public short numberOfShotsRemaining = 8;

  public boolean isRotateMotorInitialized = false;

  public CannonSubsystem(CommandXboxController controller) {
    xBoxController = controller;
    configCannonSubsys();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Foreward Limit Enabled", rotatePresetLimit.isPressed());
    SmartDashboard.putNumber("Rotation Angle in Degrees", getRotateMotorEncoder());
    SmartDashboard.putBoolean("Is rotation initialized", isRotateMotorInitialized);
    SmartDashboard.putBoolean("Arm Retract Number", loadActuatorSwitch.get());
    SmartDashboard.putNumber("Number of shots remaining", numberOfShotsRemaining);
    if (!isRotateMotorInitialized) {
      CommandScheduler.getInstance().schedule(initRotate());
    }
  }

  private Command initRotate() {
    return run(() -> {
      loadActuatorSolenoid.set(Value.kReverse);
    }).until(() -> loadActuatorSwitch.get()).andThen(() -> {
      rotateMotor.set(0.08);
    }).until(() -> rotatePresetLimit.isPressed()).andThen(() -> {
      rotateMotor.stopMotor();
      Timer.delay(0.5);
      resetRotateMotorEncoder();
      rotatePresetLimit.enableLimitSwitch(false);
      isRotateMotorInitialized = true;
    });
  }

  public double getRotateMotorEncoder() { 
    return rotateMotorEncoder.getPosition();
  }

  public void resetRotateMotorEncoder() {
    rotateMotorEncoder.setPosition(0.0);
  }

  public void setRotationAngle(double angleDegrees) {
    rotationAngleDegrees = angleDegrees;
  }

  public Command holdAngle() {
    return run(() -> {
      rotateMotorPIDController.setReference(rotationAngleDegrees, CANSparkMax.ControlType.kPosition);
    });
  }

  public Command fireCannon() {
    if (numberOfShotsRemaining > 0) {
      return runOnce(() -> {
        loadActuatorSolenoid.set(Value.kForward);
      }).until(loadActuatorSwitch::get).andThen(() -> {
        loadActuatorSolenoid.set(Value.kOff);
        firesSolenoid.set(Value.kForward);
        Timer.delay(2.0);
        firesSolenoid.set(Value.kReverse);
        Timer.delay(2.0);
        firesSolenoid.set(Value.kOff);
        numberOfShotsRemaining--;
        if (numberOfShotsRemaining == 0) {
          xBoxController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          Timer.delay(1.0);
          xBoxController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        } else {
          loadActuatorSolenoid.set(Value.kReverse);
          while (!loadActuatorSwitch.get());
          loadActuatorSolenoid.set(Value.kOff);
          setRotationAngle(rotationAngleDegrees + 45);
        }
      });
    } else {
      return runOnce(() -> {
        xBoxController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
      }).withTimeout(1.0).andThen(() -> {
        xBoxController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
      });
    }
  }

  public Command retractLoadArm() {
    return runOnce(() -> {
      loadActuatorSolenoid.set(Value.kReverse);

    });
  }

  private void configCannonSubsys() {
    // PID for rotation
    rotateMotorPIDController.setP(CannonConstants.rotateKP);
    rotateMotorPIDController.setI(CannonConstants.rotateKI);
    rotateMotorPIDController.setD(CannonConstants.rotateKD);

    // Conversion factors
    rotateMotorEncoder.setPositionConversionFactor(360 / CannonConstants.rotateMotorGearRatio);

    rotateMotor.setIdleMode(IdleMode.kBrake);
  }
}

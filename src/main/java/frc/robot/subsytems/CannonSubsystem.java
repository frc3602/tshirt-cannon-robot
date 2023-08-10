/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.robot.subsytems;

import com.fasterxml.jackson.databind.deser.SettableAnyProperty;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CannonSubsystem extends SubsystemBase {
  private CommandXboxController xBoxController;

  // Motor controllers
  private final CANSparkMax rotateMotor = new CANSparkMax(CannonConstants.rotateMotorCANID, MotorType.kBrushless);
  // private final CANSparkMax elevateMotor = new
  // CANSparkMax(CannonConstants.rotateMotorCANID, MotorType.kBrushless);

  // PID controllers
  private final SparkMaxPIDController rotateMotorPIDController = rotateMotor.getPIDController();
  // private final SparkMaxPIDController elevateMotorPIDController =
  // elevateMotor.getPIDController();

  // Motor encoders
  private final RelativeEncoder rotateMotorEncoder = rotateMotor.getEncoder();
  // private final RelativeEncoder elevateMotorEncoder =
  // elevateMotor.getEncoder();

  // Pneumatics
  private final PneumaticHub pneumaticHub = new PneumaticHub(CannonConstants.pneumaticHubCANID);
  // private final AnalogInput firePressSense = new
  // AnalogInput(CannonConstants.firePressSenseChan);
  private final DoubleSolenoid firesSolenoid = new DoubleSolenoid(CannonConstants.pneumaticHubCANID,
      PneumaticsModuleType.REVPH, CannonConstants.fillSolenoidChan, CannonConstants.firesSolenoidChan);

  public final DoubleSolenoid loadActuatorSolenoid = new DoubleSolenoid(CannonConstants.pneumaticHubCANID,
      PneumaticsModuleType.REVPH, CannonConstants.loadActuatorSolenoidFwdChan,
      CannonConstants.loadActuatorSolenoidRevChan);

  // DIO
  private final DigitalInput loadActuatorSwitch = new DigitalInput(CannonConstants.loadActuatorSwitchDIOChan);
  public final SparkMaxLimitSwitch rotatePresetLimit = rotateMotor
      .getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  // private final SparkMaxLimitSwitch elevatePresetLimit = elevateMotor
  // .getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  public double rotationAngleDegrees;
  // public double elevationAngleDegrees;
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
    if (!isRotateMotorInitialized)
      initRotateMotor();
  }

  private void initRotateMotor() {
    loadActuatorSolenoid.set(Value.kReverse);
    while (!loadActuatorSwitch.get());
    loadActuatorSolenoid.set(Value.kOff);
    rotateMotor.set(0.08);
    while (!rotatePresetLimit.isPressed());
    rotateMotor.set(0.0);
    Timer.delay(1.0);
    resetRotateMotorEncoder();
    rotatePresetLimit.enableLimitSwitch(false);
    isRotateMotorInitialized = true;
  }

  public double getRotateMotorEncoder() {
    return rotateMotorEncoder.getPosition();
  }

  /*
   * public double getElevateMotorEncoder() {
   * return elevateMotorEncoder.getPosition();
   * }
   */

  public void resetRotateMotorEncoder() {
    rotateMotorEncoder.setPosition(0.0);
  }

  /*
   * public void resetElevateMotorEncoder() {
   * elevateMotorEncoder.setPosition(0.0);
   * }
   */

  public void setRotationAngle(double angleDegrees) {
    rotationAngleDegrees = angleDegrees;
  }

  /*
   * public void setElevationAngle(double angleDegrees) {
   * elevationAngleDegrees = angleDegrees;
   * }
   */

  public Command holdAngle() {
    return run(() -> {
      rotateMotorPIDController.setReference(rotationAngleDegrees, CANSparkMax.ControlType.kPosition);
      // elevateMotorPIDController.setReference(elevationAngleDegrees,
      // CANSparkMax.ControlType.kPosition);
    });
  }

  public Command fireCannon() {
    return runOnce(() -> {
      if (numberOfShotsRemaining > 0) {
        loadActuatorSolenoid.set(Value.kForward);
        while(loadActuatorSwitch.get());
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
      } else {
        xBoxController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        Timer.delay(1.0);
        xBoxController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
      }
    });
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

    /*
     * // PID for elevation
     * elevateMotorPIDController.setP(CannonConstants.elevateKP);
     * elevateMotorPIDController.setI(CannonConstants.elevateKI);
     * elevateMotorPIDController.setD(CannonConstants.elevateKD);
     */

    // Conversion factors
    rotateMotorEncoder.setPositionConversionFactor(360 / CannonConstants.rotateMotorGearRatio);
    // elevateMotorEncoder.setPositionConversionFactor(360 /
    // CannonConstants.elevateMotorGearRatio);
    
    rotateMotor.setIdleMode(IdleMode.kBrake);
  }
}

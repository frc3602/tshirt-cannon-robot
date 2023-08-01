/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.robot.subsytems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CannonSubsystem extends SubsystemBase {
  // Motor controllers
  public final CANSparkMax rotateMotor = new CANSparkMax(CannonConstants.rotateMotorCANID, MotorType.kBrushless); // This should be private but is public for testing.
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
  private final AnalogInput firePressSense = new AnalogInput(CannonConstants.firePressSenseChan);
  private final DoubleSolenoid firesSolenoid = new DoubleSolenoid(CannonConstants.pneumaticHubCANID,
      PneumaticsModuleType.REVPH, CannonConstants.fillSolenoidChan, CannonConstants.firesSolenoidChan);

  private final DoubleSolenoid chargeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      CannonConstants.chargeSolenoidFwdChan, CannonConstants.chargeSolenoidRevChan);
  private final DoubleSolenoid loadActuatorSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      CannonConstants.loadActuatorInsertSolenoidFwdChan, CannonConstants.loadActuatorInsertSolenoidRevChan);
  private final DoubleSolenoid retSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      CannonConstants.loadActuatorRetSolenoidFwdChan, CannonConstants.loadActuatorRetSolenoidRevChan);

  // DIO
  private final DigitalInput loadActuatorRet = new DigitalInput(CannonConstants.loadActuatorRetDIOChan);
  private final DigitalInput loadActuatorInsert = new DigitalInput(CannonConstants.loadActuatorInsertDIOChan);
  private final SparkMaxLimitSwitch rotatePresetLimit = rotateMotor
      .getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  // private final SparkMaxLimitSwitch elevatePresetLimit = elevateMotor
  // .getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  public double rotationAngleDegrees;
  // public double elevationAngleDegrees;

  public CannonSubsystem() {
    configCannonSubsys();

    initRotateMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Foreward Limit Enabled", rotatePresetLimit.isPressed());
    SmartDashboard.putNumber("Rotation Angle in Degrees", getRotateMotorEncoder());
  }

  private void initRotateMotor() {
    rotateMotor.set(0.1);
    while(!rotatePresetLimit.isPressed());
    rotateMotor.stopMotor();
    resetRotateMotorEncoder();
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

  public Command holdAngles() {
    return run(() -> {
      rotateMotorPIDController.setReference(rotationAngleDegrees, CANSparkMax.ControlType.kPosition);
      // elevateMotorPIDController.setReference(elevationAngleDegrees,
      // CANSparkMax.ControlType.kPosition);
    });
  }

  public Command fireCannon() {
    return runOnce(() -> {
      firesSolenoid.set(Value.kForward);
      Timer.delay(2.0);
      firesSolenoid.set(Value.kReverse);
      Timer.delay(2.0);
      firesSolenoid.set(Value.kOff);
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
  }
}

/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.robot.subsytems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CannonSubsystem extends SubsystemBase {
  // Motor controllers
  private final CANSparkMax rotateMotor = new CANSparkMax(CannonConstants.rotateMotorCANID, MotorType.kBrushless);
  private final CANSparkMax elevateMotor = new CANSparkMax(CannonConstants.rotateMotorCANID, MotorType.kBrushless);

  // Motor encoders
  private final RelativeEncoder rotateMotorEncoder = rotateMotor.getEncoder();
  private final RelativeEncoder elevateMotorEncoder = elevateMotor.getEncoder();

  // Control system contollers
  private final PIDController rotatePIDController = new PIDController(CannonConstants.rotateKP,
      CannonConstants.rotateKI, CannonConstants.rotateKD);
  private final SimpleMotorFeedforward rotateFeedfwd = new SimpleMotorFeedforward(CannonConstants.rotateKS,
      CannonConstants.rotateKV, CannonConstants.rotateKA);
  private final PIDController elevatePIDController = new PIDController(CannonConstants.elevateKP,
      CannonConstants.elevateKI, CannonConstants.elevateKD);
  private final SimpleMotorFeedforward elevateFeedfwd = new SimpleMotorFeedforward(CannonConstants.elevateKS,
      CannonConstants.elevateKV, CannonConstants.elevateKA);

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
  private final SparkMaxLimitSwitch elevatePresetLimit = elevateMotor
      .getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  public CannonSubsystem() {
    configCannonSubsys();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getRotateMotorEncoder() {
    return rotateMotorEncoder.getPosition();
  }

  public double getElevateMotorEncoder() {
    return elevateMotorEncoder.getPosition();
  }

  public Command rotateMotor(DoubleSupplier angleSup) {
    return run(() -> {
      rotateMotor.setVoltage(rotatePIDController.calculate(getRotateMotorEncoder(), angleSup.getAsDouble())
          + rotateFeedfwd.calculate(Math.toRadians(angleSup.getAsDouble()), 0.0));
    });
  }

  public Command elevateMotor(DoubleSupplier angleSup) {
    return run(() -> {
      elevateMotor.setVoltage(elevatePIDController.calculate(getElevateMotorEncoder(), angleSup.getAsDouble())
          + elevateFeedfwd.calculate(Math.toRadians(angleSup.getAsDouble()), 0.0));
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
    rotateMotorEncoder.setPositionConversionFactor(360 / CannonConstants.rotateMotorGearRatio);
    elevateMotorEncoder.setPositionConversionFactor(360 / CannonConstants.elevateMotorGearRatio);
  }
}

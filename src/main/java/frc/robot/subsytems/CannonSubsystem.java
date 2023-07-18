/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.robot.subsytems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CannonSubsystem extends SubsystemBase {
  // private final CANSparkMax rotateMotor = new
  // CANSparkMax(CannonConstants.rotateMotorCANID, MotorType.kBrushless);
  // private final CANSparkMax elevateMotor = new
  // CANSparkMax(CannonConstants.rotateMotorCANID, MotorType.kBrushless);

  private final PneumaticHub pneumaticHub = new PneumaticHub(CannonConstants.pneumaticHubCANID);
  private final AnalogInput firePressSense = new AnalogInput(CannonConstants.firePressSenseChan);
  private final DoubleSolenoid firesSolenoid = new DoubleSolenoid(CannonConstants.pneumaticHubCANID,
      PneumaticsModuleType.REVPH, CannonConstants.fillSolenoidChan, CannonConstants.firesSolenoidChan);

  private final Solenoid chargeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, CannonConstants.chargeSolenoidChan);
  private final Solenoid loadActuatorSolenoid = new Solenoid(PneumaticsModuleType.REVPH,
      CannonConstants.loadActuatorInsertSolenoidChan);
  private final Solenoid retSolenoid = new Solenoid(PneumaticsModuleType.REVPH,
      CannonConstants.loadActuatorRetSolenoidChan);

  private final DigitalInput loadActuatorRet = new DigitalInput(CannonConstants.loadActuatorRetDIOChan);
  private final DigitalInput loadActuatorInsert = new DigitalInput(CannonConstants.loadActuatorInsertDIOChan);
  // private final SparkMaxLimitSwitch rotatePresetLimit = rotateMotor
  // .getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  // private final SparkMaxLimitSwitch elevatePresetLimit = elevateMotor
  // .getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  public CannonSubsystem() {
    configCannonSubsys();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

  }
}

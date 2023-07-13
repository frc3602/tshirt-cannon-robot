// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CannonSubsystem extends SubsystemBase {
  private final CANSparkMax rotateMotor = new CANSparkMax(CannonConstants.rotateMotorCANID, MotorType.kBrushless);
  private final CANSparkMax elevateMotor = new CANSparkMax(CannonConstants.rotateMotorCANID, MotorType.kBrushless);

  private final PneumaticHub pneumaticHub = new PneumaticHub(CannonConstants.pneumaticHubCANID);
  private final AnalogInput firePressSense = new AnalogInput(CannonConstants.firePressSenseChan);
  private final Solenoid firesSolenoid = new Solenoid(PneumaticsModuleType.REVPH, CannonConstants.firesSolenoidChan);
  private final Solenoid chargeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, CannonConstants.chargeSolenoidChan);
  private final Solenoid loadActuatorSolenoid = new Solenoid(PneumaticsModuleType.REVPH, CannonConstants.loadActuatorInsertSolenoidChan);
  private final Solenoid retSolenoid = new Solenoid(PneumaticsModuleType.REVPH, CannonConstants.loadActuatorRetSolenoidChan);
  

  private final DigitalInput loadActuatorRet = new DigitalInput(CannonConstants.loadActuatorRetDIOChan);
  private final DigitalInput loadActuatorInsert = new DigitalInput(CannonConstants.loadActuatorInsertDIOChan);
  private final SparkMaxLimitSwitch rotatePresetLimit = rotateMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  private final SparkMaxLimitSwitch elevatePresetLimit = elevateMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  

  
  /** Creates a new DriveSubsytem. */
  public CannonSubsystem() {
    configCannonSubsys();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void configCannonSubsys() {

  }
   
}

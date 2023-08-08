/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.robot;

public final class Constants {
  public static final class DriveConstants {
    public static final int xboxControllerPort = 0;

    // Motor CAN Ids
    public static final int frontLeftMotorCANID = 1;
    public static final int backLeftMotorCANID = 2;
    public static final int frontRightMotorCANID = 3;
    public static final int backRightMotorCANID = 4;
  }

  public static final class CannonConstants {
    // CAN Ids
    public static final int rotateMotorCANID = 6; // Actually 5 lol
    // public static final int elevateMotorCANID = 6;

    // Gear ratios
    public static final double rotateMotorGearRatio = 0.0;
    // public static final double elevateMotorGearRatio = 0.0;

    // Rotate PID gains
    public static final double rotateKP = 0.0;
    public static final double rotateKI = 0.0;
    public static final double rotateKD = 0.0;

    // Rotate feedforeward gains
    public static final double rotateKS = 0.0;
    public static final double rotateKV = 0.0;
    public static final double rotateKA = 0.0;

    /*
     * // Elevate PID gains
     * public static final double elevateKP = 0.0;
     * public static final double elevateKI = 0.0;
     * public static final double elevateKD = 0.0;
     * 
     * // Elevate feedforeward gains
     * public static final double elevateKS = 0.0;
     * public static final double elevateKV = 0.0;
     * public static final double elevateKA = 0.0;
     */

    // Pmeumatic Ids
    public static final int pneumaticHubCANID = 7;
    public static final int firePressSenseChan = 0;
    public static final int firesSolenoidChan = 0;
    public static final int fillSolenoidChan = 4;

    public static final int chargeSolenoidFwdChan = 1;
    public static final int chargeSolenoidRevChan = 5;

    public static final int loadActuatorSolenoidFwdChan = 2;
    public static final int loadActuatorSolenoidRevChan = 3;

    // Limit switch channels
    public static final int loadActuatorSwitchDIOChan = 0;

  }
}

/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.robot;

public final class Constants {
    public static final class DriveConstants {
        public static final int xboxControllerPort = 0;

        public static final int frontLeftMotorCANID = 1;
        public static final int backLeftMotorCANID = 2;
        public static final int frontRightMotorCANID = 3;
        public static final int backRightMotorCANID = 4;
    }

    public static final class CannonConstants {
        public static final int rotateMotorCANID = 5;
        public static final int elevateMotorCANID = 6;

        public static final int pneumaticHubCANID = 7;
        public static final int firePressSenseChan = 0;
        public static final int firesSolenoidChan = 0;
        public static final int fillSolenoidChan = 4;

        public static final int chargeSolenoidChan = 1;

        public static final int loadActuatorRetDIOChan = 0;
        public static final int loadActuatorInsertDIOChan = 1;
        // These are on the Pneumatic Hub
        public static final int loadActuatorRetSolenoidChan = 2;
        public static final int loadActuatorInsertSolenoidChan = 3;
    }
}

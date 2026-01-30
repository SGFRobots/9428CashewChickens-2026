// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {        
    // public static final String ModuleNameSim = "Swerve Modules"; // DELETED IN THE GREAT RONIN CLEAN UP OF 2026
    public static final boolean fieldOriented = true;
    
    public static final class MotorPorts { // MOTOR PORTS ALL CHANGE EXCPET GYRO + LIMELIHGT 
        // CAN IDs of driving motors
        public static final int kFLDriveMotorID = 1; 
        public static final int kBLDriveMotorID = 4;
        public static final int kFRDriveMotorID = 2;
        public static final int kBRDriveMotorID = 3;

        // CAN IDs of turning motors
        public static final int kFLTurningMotorID = 1;
        public static final int kBLTurningMotorID = 4;
        public static final int kFRTurningMotorID = 2;
        public static final int kBRTurningMotorID = 3;

        // CAN IDs of CANCoders
        public static final int kFLDriveAbsoluteEncoderID = 5;
        public static final int kBLDriveAbsoluteEncoderID = 8;
        public static final int kFRDriveAbsoluteEncoderID = 6;
        public static final int kBRDriveAbsoluteEncoderID = 7;
        // 1 4 2 3 5 8 6 7 
        // Gyro
        public static final int kGyroPort = 2;

        // LEDs
        public static final int LEDChannel = 8;

        // Limelight
        public static final String kLeftLimelightKey = "limelight-left";
        public static final String kRightLimelightKey = "limelight-right";
    }

    // Reversed motors
    public static final class Reversed { // These might change! 
        // Turning motors
        public static final boolean kFLTurningReversed = true;
        public static final boolean kBLTurningReversed = true;
        public static final boolean kFRTurningReversed = true;
        public static final boolean kBRTurningReversed = true;
    
        // Driving motors
        public static final boolean kFLDriveReversed = false;
        public static final boolean kBLDriveReversed = false;
        public static final boolean kFRDriveReversed = false;
        public static final boolean kBRDriveReversed = false;

        // Turning encoders
        public static final boolean kFLTurningEncoderReversed = true;
        public static final boolean kBLTurningEncoderReversed = true;
        public static final boolean kFRTurningEncoderReversed = true;
        public static final boolean kBRTurningEncoderReversed = true;
    
        // Driving encoders
        public static final boolean kFLDriveEncoderReversed = false;
        public static final boolean kBLDriveEncoderReversed = false;
        public static final boolean kFRDriveEncoderReversed = false;
        public static final boolean kBRDriveEncoderReversed = false;
        
        // CANCoders
        public static final boolean kFLDriveAbsoluteEncoderReversed = false;
        public static final boolean kBLDriveAbsoluteEncoderReversed = false;
        public static final boolean kFRDriveAbsoluteEncoderReversed = false;
        public static final boolean kBRDriveAbsoluteEncoderReversed = false;
    }

    // Physical and mechanical variables
    public static final class Mechanical { // DOUBLE CHECK LATER

        // Robot's physical measurements
        public static final double kWheelRadiusMeters = 0.0508;
        public static final double kWheelDiameterMeters = kWheelRadiusMeters * 2;
        public static final double kDriveMotorGearRatio = 6.12;
        public static final double kTurningMotorGearRatio = 150/7;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kDriveEncoderResolution = 2048;
        public static final double kTurningEncoderResolution = 42;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kEncoderCPR = 1024;
        public static final double kDistancePerPulse = (kWheelCircumferenceMeters) / kEncoderCPR;
        // The SysId tool provides a convenient method for obtaining these values for your robot.
        // the values being the volt related lines below 
        public static final double kVoltSecondPerRadian = 3.41;
        public static final double kVoltSecondsSquaredPerRadian = 0.111;
        public static final DCMotor kDriveGearBox = DCMotor.getKrakenX60(1);
        public static final DCMotor kTurnGearBox = DCMotor.getKrakenX60(1);

        
        // Distance between right and left wheels (in meters)
        public static final double kRobotWidthMeters = 0.6477; //change
        // Distance between front and back wheels (in meters)
        public static final double kRobotLengthMeters = 0.6477; //changed to 25.5 inches. was at 26.5 inches.

        // CANCoders' offsets
        public static final double kFLDriveAbsoluteEncoderOffset = -0.0483; 
        public static final double kBLDriveAbsoluteEncoderOffset = 0.2954;
        public static final double kFRDriveAbsoluteEncoderOffset = 0.0139; 
        public static final double kBRDriveAbsoluteEncoderOffset = 0.4727; //change change change and change 

        // Module Positions on Robot
        public static final Translation2d[] kModulePositions = {
            new Translation2d(kRobotLengthMeters / 2, kRobotWidthMeters / 2),
            new Translation2d(kRobotLengthMeters / 2, -kRobotWidthMeters / 2),
            new Translation2d(-kRobotLengthMeters / 2, kRobotWidthMeters / 2),
            new Translation2d(-kRobotLengthMeters / 2, -kRobotWidthMeters / 2) // should stay same 
        };
        // Kinematics map of robot
        // FL, FR, BL, BR
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            kModulePositions[0],
            kModulePositions[1],
            kModulePositions[2],
            kModulePositions[3]); // good - tiny t
        
        // Deadzone // maybe change ? 
        public static final double kDeadzone = 0.1;
        public static final double AprilTagDeadzone = 3;
    
        // Speeds and Accelerations // check 
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.0292;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 12.119;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 10;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 10;

    }

    public static final class AprilTags {
        // x, distance, yaw
        public static final double[] leftCoral = {0.04, 0.23, 1};
        // public static final double[] leftCoral = {2.52, 30.11, 1};
        public static final double[] rightCoral = {0, 0.28, 1};
        // public static final double[] rightCoral = {-4.35, 16.98, 2};

        public static final double xErrorAllowed = 0.65;
        public static final double distanceErrorAllowed = 0.03;
        public static final double yawErrorAllowed = 1;
    }

    public static final class PIDAlignment {
        public static final PIDController leftXPID = new PIDController(0.8, 0, 0);
        public static final PIDController leftYPID = new PIDController(0.6, 0, 0);
        public static final PIDController leftTurnPID = new PIDController(0.03, 0, 0);

        public static final PIDController rightXPID = new PIDController(0.8, 0, 0);
        public static final PIDController rightYPID = new PIDController(0.6, 0, 0);
        public static final PIDController rightTurnPID = new PIDController(0.03, 0, 0);
    }

    // Controller ports
    public static final class Controllers {
        // Controller ports
        public static final int DrivingControllerPort = 0;
        public static final int XBoxControllerPort = 1;
        public static final int BackupDriveControllerPort = 2;

        // New Controllers
        public static final class DrivingController {
            // Joystick
            public static final int RightXPort = 0;
            public static final int RightYPort = 1;
            public static final int LeftXPort = 3;
            public static final int LeftYPort = 2;

            // Bumpers? 1 -> down; -1 -> up
            public static final int LeftHoldBtn = 4;
            public static final int RightHoldBtn = 6;

            // Switches: 1 -> up; 0 -> middle; -1 -> down
            public static final int LeftSwitch = 5;
            public static final int RightSwtich = 7;

            // Buttons
            public static final int LeftButton = 1;
            public static final int Dial = 2; // right = on ??? idk its weird
        }

        // Black drone controller
        public static final class OldController {
            // Joysticks and triggers
            public static final int LeftXPort = 0;
            public static final int LeftYPort = 1;
            public static final int RightXPort = 2;
            public static final int RightYPort = 3;
            
            // Dials
            public static final int s1DialPort = 4;
            public static final int s2DialPort = 5;
            
            // Switches
            public static final int SwitchE = 6;
            public static final int SwitchF = 7;
            public static final int UpperB = 1;
            public static final int MiddleB = 2;
            public static final int LowerB = 3;
            public static final int UpperC = 4;
            public static final int MiddleC = 5;
            public static final int LowerC = 6;
            
            // Buttons
            public static final int ButtonAPort = 7;
            public static final int ButtonDPort = 8;
        }

        public static final class XBoxBackup {
            public static final int LeftXPort = 0;
            public static final int LeftYPort = 1;
            public static final int RightXPort = 4;
            public static final int RightYPort = 5;

            public static final int ButtonA = 1;
            public static final int ButtonB = 2;
            public static final int ButtonX = 3;
            public static final int ButtonY = 4;
        }
  }
}
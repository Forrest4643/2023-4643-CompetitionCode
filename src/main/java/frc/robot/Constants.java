// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2023 FRC season for team 4643, Butte Built Bots

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class Constants {
    public static class driveConstants{
        public static final int kLeftFrontID = 4;
        public static final int kLeftRearID = 3;
        public static final int kRightFrontID = 1;
        public static final int kRightRearID = 2;   
        public static final double kInputDeadband = 0.08;

        private static final Translation2d kFrontLeftWheelMeters = new Translation2d(0.302578, 0.254000);
        private static final Translation2d kFrontRightWheelMeters = new Translation2d(0.302578, -0.254000);
        private static final Translation2d kRearLeftWheelMeters = new Translation2d(-0.302578, 0.254000);
        private static final Translation2d kRearRightWheelMeters = new Translation2d(-0.302578, -0.254000);

        public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(kFrontLeftWheelMeters, kFrontRightWheelMeters, kRearLeftWheelMeters, kRearRightWheelMeters); 

        public static final int kDriveSparkSmartCurrentLimit = 40; //Amps
        
        public static final double steerkP = 0.085;
        public static final double steerkI = 0.0;
        public static final double steerkD = 0.00001;
        public static final double steerkF = 0.15;

        public static final double kWheelRPMaccel = 750;

        public static final double kMaxAttainableMetersPerSecond = 4.5;

        public static final double kMaxWheelRPM = 5700 / 7;

        public static final double kWheelCircumferenceMeters = 0.4787787204;
        public static final double kAllowedVelocityErrorRPM = 5;
        public static final double kVelocityConversionFactorRPM = 0.14285714285; //7:1 versaplanetary
        public static final double kPositionConversionFactorMeters = kWheelCircumferenceMeters / 7; //0.0762m (3in) wheel radius

        //multiplier for drive controller sin function, this is used to get a nicer response curve from the controller
        public static final double kSpeedSinMultiplier = 1.188;

    }

    public static final class armConstants {
        public static final int kArmID = 5;

        public static final double kMaxVelocityDegSec = 90;
        public static final double kMaxAccelDegSec = 45;

        public static final double kUnStow1 = -20;
        public static final double kUnStow2 = -80;
        public static final double kMatchStow = -70;
        public static final double kIntake = -35;
        public static final double kHorizontal = 0;

        public static final double kScoreLowCube = -45;
        public static final double kScoreMidCube = -10;
        public static final double kScoreHighCube = 10;

        public static final double kScoreLowCone = -60;
        public static final double kScoreMidCone = -3;
     
        public static final double kSubstationPos = 15;

    }

    public static final class telescopingConstant {
        public static final int kTeleID = 6;

        public static final double kScoreLowCube = 0;
        public static final double kScoreMidCube = 6;
        public static final double kScoreHighCube = 12.5;

        public static final double kScoreLowCone = 0;
        public static final double kScoreMidCone = 4;

        public static final double kStow = 0;

        public static final double kSubstation = 0;

        public static final double kMaxPositionIN = 11.5;
        public static final double kMinPositionIN = 0;


    }

    public static final class wristConstants {
        public static final int kWristID = 7;
        public static final double kMaxVelocityMeters = 0;
        public static final double kMaxAccelMeter = 0;

        public static final double kUnStow = 90;
        public static final double kHoldForUnstow = -80;
        public static final double kMatchStow = 90;
        public static final double kHorizontal = 0;
        public static final double kIntake = 0;

        public static final double kScoreLowCubePos = 40;
        public static final double kScoreMidCubePos = 15;
        public static final double kScoreHighCubePos = 15;

        public static final double kScoreLowConePos = 40;
        public static final double kScoreMidConePos = 45;

        public static final double kSubstationPos = -15;

        public static final double kWristMinPositionDEG = -120;
        public static final double kWristMaxPositionDEG = 120;

        public static final double kWristSensitivity = 2;

    }

    public static final class mandibleConstants {
        public static final int kMandibleID = 8;

        public static final double kHoldVoltage = 0.5;

        public static final double kCurrentThresh = 19;
    }
 
    public static final class visionConstants {
        public static final Transform3d kRobotToFrontCamMeters = new  Transform3d(
            new Translation3d(0.052878, 0.177 , 0.763443), 
            new Rotation3d(-10, 0, 0));
    }

    public static class autoConstants {
        

        public static final double kPXController = 7.0;
        public static final double kIXController = 0.0;
        public static final double kDXController = 0.0;

        public static final double kPYController = 12.0;
        public static final double kIYController = 0.0005;
        public static final double kDYController = 0.001;

        public static final double kPThetaController = 0.05;
        public static final double kIThetaController = 0.0;
        public static final double kDThetaController = 0.0;

        public static final PIDController kxPID = new PIDController(kPXController, kIXController, kDXController);
        public static final PIDController kyPID = new PIDController(kPYController, kIYController, kDYController);
        public static TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(2.0, 2.0);
        public static final ProfiledPIDController kthetaPID = new ProfiledPIDController(kPThetaController, kIThetaController, kDThetaController, kThetaControllerConstraints);
        
        public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(0.3, 3.5);

        public static final double kMaxSpeedMetersPerSecond = 2.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
       

    }
}

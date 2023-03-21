// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

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
import io.github.oblarg.oblog.Loggable;


public class Constants implements Loggable {
    public static class driveConstants implements Loggable{
        public static final int kLeftFrontID = 1;
        public static final int kLeftRearID = 2;
        public static final int kRightFrontID = 4;
        public static final int kRightRearID = 3;
        public static final double kInputDeadband = 0.08;

        private static final Translation2d kFrontLeftWheelMeters = new Translation2d(0.302578, 0.254000);
        private static final Translation2d kFrontRightWheelMeters = new Translation2d(0.302578, -0.254000);
        private static final Translation2d kRearLeftWheelMeters = new Translation2d(-0.302578, 0.254000);
        private static final Translation2d kRearRightWheelMeters = new Translation2d(-0.302578, -0.254000);

        public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(kFrontLeftWheelMeters, kFrontRightWheelMeters, kRearLeftWheelMeters, kRearRightWheelMeters); 


        public static final int kDriveSparkSmartCurrentLimit = 50; //Amps
        
        public static final double steerkP = 0.15;
        public static final double steerkI = 0.0;
        public static final double steerkD = 0.0000075;
        public static final double steerkF = 0.1;

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

        public static final double kScoreLowCubePos = 25;
        public static final double kScoreMidCubePos = 50;
        public static final double kScoreHighCubePos = 90;

        public static final double kScoreLowConePos = 30;
        public static final double kScoreMidConePos = 50;
     
        public static final double kStowPos = 0;

        public static final double kSubstationPos = 60;

    }

    public static final class telescopingConstant {
        public static final int kTeleID = 6;

        public static final double kScoreLowCubePos = 0;
        public static final double kScoreMidCubePos = 0;
        public static final double kScoreHighCubePos = 0;

        public static final double kScoreLowConePos = 0;
        public static final double kScoreMidConePos = 0;

        public static final double kStowPos = 0;

        public static final double kSubstationPos = 0;

    }

    public static final class wristConstants {
        public static final int kWristID = 7;
        public static final double kMaxVelocityMeters = 0;
        public static final double kMaxAccelMeter = 0;

        public static final double kScoreLowCubePos = 90;
        public static final double kScoreMidCubePos = 100;
        public static final double kScoreHighCubePos = 150;

        public static final double kScoreLowConePos = 60;
        public static final double kScoreMidConePos = 120;

        public static final double kSubstationPos = 120;

        public static final double kWristMinPositionDEG = 0;
        public static final double kWristMaxPositionDEG = 240;

        public static final double kWristSensitivity = 2;


    }

    public static final class mandibleConstants {
        public static final int kMandibleID = 8;

        public static final double kHoldVoltage = 3;

        public static final double kCurrentThresh = 10;
    }
 
    public static final class visionConstants {
        public static final Transform3d kRobotToFrontCamMeters = new  Transform3d(
            new Translation3d(-0.177, -0.052878, 0.763443), 
            new Rotation3d(0, -5, 0));
    }

    public static class autoConstants implements Loggable {
        

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

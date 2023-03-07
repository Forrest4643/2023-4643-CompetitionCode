// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;


public class Constants implements Loggable {
    public static class driveConstants implements Loggable{
        public static final int leftFrontID = 1;
        public static final int leftRearID = 2;
        public static final int rightFrontID = 4;
        public static final int rightRearID = 3;
        public static final double inputDeadband = 0.08;

        private static final Translation2d frontLeftWheelMeters = new Translation2d(-0.302578, 0.254000);
        private static final Translation2d frontRightWheelMeters = new Translation2d(0.302578, 0.254000);
        private static final Translation2d rearLeftWheelMeters = new Translation2d(-0.302578, -0.254000);
        private static final Translation2d rearRightWheelMeters = new Translation2d(0.302578, -0.254000);

        public static final MecanumDriveKinematics driveKinematics = new MecanumDriveKinematics(frontLeftWheelMeters, frontRightWheelMeters, rearLeftWheelMeters, rearRightWheelMeters); 


        public static final int driveSparkSmartCurrentLimit = 50; //Amps
        
        public static final double steerkP = 0.04;
        public static final double steerkI = 0.0;
        public static final double steerkD = 0.00001;
        public static final double steerkF = 0.06;

        public static final double wheelRPMaccel = 750;

        public static final double maxAttainableMetersPerSecond = 4.5;

        public static final double wheelCircumferenceMeters = 0.4787787204;

        public static final double velocityConversionFactorRPM = 0.14285714285; //7:1 versaplanetary
        public static final double positionConversionFactorMeters = wheelCircumferenceMeters / 10; //0.0762m (3in) wheel radius

        //multiplier for drive controller sin function, this is used to get a nicer response curve from the controller
        public static final double speedSinMultiplier = 1.188;


    }

    public static final class armConstants {
        public static final int armID = 5;
        public static final int teleID = 6;

        public static double maxVelocityDegSec = 90;
        public static double maxAccelDegSec = 45;
     
    }

    public static final class wristConstants {
        public static final int wristID = 7;
        public static final double maxVelocityMeters = 0;
        public static final double maxAccelMeters = 0;

    }

    public static final class mandibleConstants {
        public static final int mandibleID = 8;
    }
 
    public static final class visionConstants {
        public static final Transform3d robotToFrontCamMeters = new  Transform3d(
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

        public static final double kPwheelVel = 0.02;
        public static final double kIwheelVel = 0.0;
        public static final double kDwheelVel = 0.0;
        
        public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(0.3, 3.5);

        public static final double kMaxSpeedMetersPerSecond = 2.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
       

        public static TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(2.0, 2.0);
    }
}

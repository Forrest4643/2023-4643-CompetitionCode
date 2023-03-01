// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public final class Constants {
    //Drive Constants
    public static final class dConstants implements Loggable{
        public static final int leftFrontID = 1;
        public static final int leftRearID = 2;
        public static final int rightFrontID = 4;
        public static final int rightRearID = 3;
        public static final double inputDeadband = 0.08;

        public static final Translation2d frontLeftWheel = new Translation2d(0.324, 0.219);
        public static final Translation2d frontRightWheel = new Translation2d(0.324, -0.219);
        public static final Translation2d rearLeftWheel = new Translation2d(-0.324, 0.219);
        public static final Translation2d rearRightWheel = new Translation2d(-0.324, -0.219);
        public static final int driveSparkSmartCurrentLimit = 50; //Amps

        public static final double sparkMAXVelocitykP = 0.00015;
        public static final double sparkMAXVelocitykI = 0;
        public static final double sparkMAXVelocitykD = 0.0;
        public static final double sparkMAXVelocitykF = 0.0025;

        public static final double steerkP = 0.04;
        public static final double steerkI = 0.0;
        public static final double steerkD = 0.00001;
        public static final double steerkF = 0.06;


        public static final double wheelRPMaccel = 750;

        public static final double velocityConversionFactor = 0.1; //10:1 versaplanetary
        public static final double positionConversionFactor = 0.04787787204; //0.0762m wheel radius

    }

    public static final class aConstants {
        public static final int armID = 5;
        public static final int teleID = 6;
     

    }
    //Pneumatic constants
    public static final class pConstants {
        public static final int compressorID = 0;

    }
    //Vision constants
    public static final class vConstants {
        public static final double cameraHeightM = Units.inchesToMeters(37.650);
        public static final double targetGroundHeightM = Units.inchesToMeters(104.000);
        public static final double cameraAngleRAD = Units.degreesToRadians(55);
        public static final double camDiagFOV = 170.0; // degrees - assume wide-angle camera
        public static final double maxLEDRange = 20.0; // meters
        public static final int camResolutionWidth = 640; // pixels
        public static final int camResolutionHeight = 480; // pixels
        public static final double minTargetAreaPIX = 10; // square pixels

    }
    //Auto constants
    public static final class autoConstants {

        public static final double kPXController = 7;
        public static final double kPYController = 12;
        public static final double kPThetaController = 0.05;
        public static final double kIThetaController = 0.0;
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kIXController = 0;
        public static final double kDXController = 0;
        public static final double kIYController = 0.0005;
        public static final double kDYController = 0.001;
        public static final double kPwheelVel = 0.02;
        public static final double kIwheelVel = 0.0;
        public static final double kDwheelVel = 0.0;
        
        
        public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(.3, 3.5);
        public static final double kMaxAccelerationMetersPerSecondSquared = .5;

        public static TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(2, 2);
    }
}

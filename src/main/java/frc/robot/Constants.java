// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class DriveConstants {
        public static final int leftFrontID = 2;
        public static final int leftRearID = 3;
        public static final int rightFrontID = 13;
        public static final int rightRearID = 12;
        public static final double stickDB = 0.05;
        public static final double turnPow = 1;
        public static final double speedPow = 1.2;
        public static final double speedSin = 1.5;
        public static final double turnSin = 1;

        public static final double drivekP = 0.5;
        public static final double drivekI = 0.004;
        public static final double drivekD = .001;

        public static final double bangTol = .25;

        //TODO sysid
        public static final double velocityConversionFactor = .0007983;
        public static final double positionConversionFactor = 0.0479;
        public static final double kaVoltSecondsSquaredPerMeter = 0;
        public static final double ksVolts = 0.0;
        public static final double kvVoltSecondsPerMeter = .05;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.7112);
        public static final double kPDriveVel = 1;
        public static final double kIDriveVel = 0;
        public static final double kDDriveVel = 0.2;
        public static final double steerkP = 0.05;
        public static final double steerkI = 0.0008;
        public static final double steerkD = 0.005;

        public static final double driveSlew = 1.25; //TODO test.
        public static final double turnSlew = 5; //TODO test.

        public static final double autoDist = -13;

        public static final double driveConversion = 0.1;
        public static final double driveInchConv = 0.0523598775598;
    }

    public static final class IndexerConstants {

       
        public static final double thresh1 = 1;
        public static final double topThresh = 0;
        public static final double bottomThresh = 0;
        public static final double blueThresh = .33;
        public static final double redThresh = .33;
        public static final double oneBall = 20;
        public static final double primeShot = -10;
        public static final double bangTolerance = .75;
        public static final int frontID = 6;
        public static final int rearID = 8;

    }

    public static final class IntakeConstants {
        public static final int frontID = 7;
        public static final int rearID = 9;

        public static final double frontRevCurrent = 25;
        public static final double rearRevCurrent = 25;


    }

    public static final class PNConstants {
        public static final int compressorID = 0;
        public static final int frontForwardID = 2;
        public static final int frontReverseID = 1;
        public static final int rearForwardID = 4;
        public static final int rearReverseID = 3;

    }

    public static final class ShooterConstants {
        public static final int leftMotorID = 10;
        public static final int rightMotorID = 1;

        public static final double lowGoal = 2000;
        public static final double highGoal = 5600;
        public static final double ejectCargo = 1000;

        public static final double voltsSlew = 3;
        public static final double kP = 0.001;
        public static final double kI = 0.0001;
        public static final double kD = 0.0;
        public static final double PIDtolerance = 10;
        public static final double kV = 0.00215;
        public static final double kA = 0;
        public static final double kS = 0;
        public static final double acc = 3746;

        public static final double quadAimA = -10.5;
        public static final double quadAimB = 4.1;
        public static final double quadAimC = -0.061;
        public static final double quadAimD = 1515;
        public static final double efficiencyConversion = 1;
        public static final double hubShot = 575;

    }

    public static final class HoodConstants {
        public static final int hoodID = 4;

        public static final double conversionFactor = 2.513274123;
        public static final double ForwardLimit = 4.723; //TODO test.
        public static final double ReverseLimit = 0.1; //TODO test.
        public static final double PIDtolerance = 0.05;
        public static final double lowAngleLimit = 74.429; //TODO test.
        public static final double highAngleLimit = 52.429; //TODO test.

        public static final double kP = .2;
        public static final double kI = 0.02;
        public static final double kD = 0.001;

        // public static final double AccInPerSec = 10;
        // public static final double InPerSec = 19.59;
        // public static final double kS = 0;
        // public static final double kG = 0.02;
        // public static final double kV = 0.05;
        // public static final double kA = 0;

        public static final double lowGoal = 3;
        public static final double highGoal = 0;
        public static final double hubShot = 53;

        public static final double quadAimA = -4.57;
        public static final double quadAimB = 0.0865;
        public static final double quadAimC = 92.4;

    }

    public static final class TurretConstants {
        public static final int turretID = 11;
        public static final double turretTicksToDegrees = 1.72193877551;
        public static final double turretForwardLimit = 95; //TODO test.
        public static final double turretReverseLimit = -95; //TODO test.
        public static final double turretLeftWarning = turretForwardLimit - 5;
        public static final double turretRightWarning = turretReverseLimit + 5;
        public static final double turretkP = 0.08;
        public static final double turretkI = 0;
        public static final double turretkD = 0.003;
        public static final double tolerance = .5;
        public static final double zeroThresh = 1;
        public static final double HUBposition = -90;

    }

    public static final class VisionConstants {
        public static final double cameraHeightM = Units.inchesToMeters(37.650);
        public static final double targetGroundHeightM = Units.inchesToMeters(104.000);
        public static final double cameraAngleRAD = Units.degreesToRadians(55);
        public static final double camDiagFOV = 170.0; // degrees - assume wide-angle camera
        public static final double maxLEDRange = 20.0; // meters
        public static final int camResolutionWidth = 640; // pixels
        public static final int camResolutionHeight = 480; // pixels
        public static final double minTargetAreaPIX = 10; // square pixels

        public static final double distA = -0.00168;
        public static final double distB = 0.819;
        public static final double distC = -25.3;

        public static final double cameraHeightMETERS = Units.inchesToMeters(37.849256);
        public static final double targetHeightMETERS = Units.inchesToMeters(104);

      
        public static final double targetWidthM = Units.inchesToMeters(53.750);
      
        public static final double targetHeightM = Units.inchesToMeters(2.000);
      
        public static final double tgtXPos = Units.inchesToMeters(312.000);
        public static final double tgtYPos = Units.inchesToMeters(164.000);
               
    }

    public static final class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = 3.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class ClimberConstants {
        public static final int climbID = 14;
        public static final double maxLimit = 5.25; //TODO review value
        public static final double minLimit = .1;
        public static final double conversionFactor = (1/48); //TODO test.
    }
}

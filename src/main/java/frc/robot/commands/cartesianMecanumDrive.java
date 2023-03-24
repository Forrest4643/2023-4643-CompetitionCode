// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.driveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Sensors;
import io.github.oblarg.oblog.annotations.Log;

public class cartesianMecanumDrive extends CommandBase  {

  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier speedX, speedY, driverHeadingAdjustment;
  private SlewRateLimiter turnSlewRateLimiter = new SlewRateLimiter(1, -50, 0);

  private PIDController driveHeadingController = new PIDController(driveConstants.steerkP, driveConstants.steerkI,
      driveConstants.steerkD, 0.02);

  //static constant
  private double headingkS = 0.06;
  //velocity constant
  private double headingkV = 0.0;
  //acceleration constant
  private double headingkA = 0.0;


  private SimpleMotorFeedforward driveHeadingFF = new SimpleMotorFeedforward(headingkS, headingkV, headingkA);

  private Sensors m_sensors;

  private double m_expectedHeading;

  private int m_rotationSelect = 0;

  private double m_steerSensitivity = 4;

  @Log
  private String rotationMode = "COR: Centered";

  private Translation2d m_COR;

  /** Creates a new cartesianMecanumDrive. */
  public cartesianMecanumDrive(DriveSubsystem m_driveSubsystem, Sensors m_Sensors, DoubleSupplier speedX,
      DoubleSupplier speedY, DoubleSupplier driverHeadingAdjustment) {
    this.m_driveSubsystem = m_driveSubsystem;
    this.m_sensors = m_Sensors;
    this.speedX = () -> Math.sin(MathUtil.applyDeadband(speedX.getAsDouble(), driveConstants.kInputDeadband) * driveConstants.kSpeedSinMultiplier);
    this.speedY = () -> Math.sin(MathUtil.applyDeadband(speedY.getAsDouble(), driveConstants.kInputDeadband) * driveConstants.kSpeedSinMultiplier);
    this.driverHeadingAdjustment = driverHeadingAdjustment;
    m_expectedHeading = 0;

    driveHeadingController.enableContinuousInput(0, 360);
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dha = driverHeadingAdjustment.getAsDouble();
    double dhaOUT;
    // maintaining minus sign
    if (dha < 0) {
      dhaOUT = -MathUtil.applyDeadband(
          Math.sin(
              turnSlewRateLimiter.calculate(Math.abs(dha))),
          driveConstants.kInputDeadband);
    } else {
      dhaOUT = MathUtil.applyDeadband(
          Math.sin(
              turnSlewRateLimiter.calculate(Math.abs(dha))),
          driveConstants.kInputDeadband);
    }

    // Takes input from the driver and adjusts the robot's expected heading
    m_expectedHeading = MathUtil.inputModulus(m_expectedHeading + (dhaOUT * m_steerSensitivity), 0, 360);

    // sending heading to PID controller
    double rotationOutput = driveHeadingController.calculate(m_sensors.wrappedNavXHeading(), m_expectedHeading)
        + driveHeadingFF.calculate(driveHeadingController.getPositionError());

    switch (m_rotationSelect) {
      case 1:
        // Forward center of rotation
        m_COR = new Translation2d(0, .5);
        rotationMode = "COR: Front";
        break; 
      case 2:
        // Left center of rotation
        m_COR = new Translation2d(-.5, 0); 
        rotationMode = "COR: Left";
        break;
      case 3:
        // Right center of rotation
        m_COR = new Translation2d(.5, 0); 
        rotationMode = "COR: Right";
        break;
      case 4:
        // Back center of rotation
        m_COR = new Translation2d(0, -.5); 
        rotationMode = "COR: Back";
        break;
      default:
        // Default to centered
        m_COR = new Translation2d(0, 0); 
        rotationMode = "COR: Centered";
        break;
    }

    // sending outputs to drive controller
    m_driveSubsystem.cartesianMecanumDrive(speedX, speedY, () -> rotationOutput, () -> m_COR.getX(), () -> m_COR.getY());

    // debug info
    SmartDashboard.putNumber("speedX", speedX.getAsDouble());
    SmartDashboard.putNumber("speedY", speedY.getAsDouble());
    SmartDashboard.putNumber("rotationOutput", rotationOutput);
    SmartDashboard.putNumber("rotationSpeed", driverHeadingAdjustment.getAsDouble());
    SmartDashboard.putNumber("expectedHeading", m_expectedHeading);

  }

  public void frontRotation() {
    m_rotationSelect = 1;
    System.out.println("Front center of rotation selected!");

  }

  public void backRotation() {
    m_rotationSelect = 4;
    System.out.println("Back center of rotation selected!");

  }

  public void leftRotation() {
    m_rotationSelect = 2;
    System.out.println("Left center of rotation selected!");

  }

  public void rightRotation() {
    m_rotationSelect = 3;
    System.out.println("Right center of rotation selected!");

  }

  public void centeredRotation() {
    m_rotationSelect = 0;
    System.out.println("Centered center of rotation selected!");

  }

  public void zeroHeading() {
    m_sensors.zeroNavXHeading();
    m_expectedHeading = 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

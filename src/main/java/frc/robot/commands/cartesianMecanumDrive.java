// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Sensors;


public class cartesianMecanumDrive extends CommandBase {

  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier speedX, speedY, rotationSpeed;
  private PIDController driveHeadingPIDController = new PIDController(0.01, 0.0, 0.0004);
  private Sensors m_sensors;

  private double m_expectedHeading;


  /** Creates a new cartesianMecanumDrive. */
  public cartesianMecanumDrive(DriveSubsystem m_driveSubsystem, Sensors m_sensors, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier driverHeadingAdjustment) {
    this.m_driveSubsystem = m_driveSubsystem;
    this.m_sensors = m_sensors;
    this.speedX = speedX;
    this.speedY = speedY;
    this.driverHeadingAdjustment = driverHeadingAdjustment;

    m_expectedHeading = 0;

    driveHeadingPIDController.enableContinuousInput(0, 360);
    addRequirements(m_driveSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Takes input from the driver and adjusts the robot's expected heading
    m_expectedHeading = m_expectedHeading + MathUtil.applyDeadband(driverHeadingAdjustment.getAsDouble(), DriveConstants.inputDeadband);    

    //sending heading to PID controller
    double rotationOutput = driveHeadingController.calculate(m_sensors.navXYaw(), m_expectedHeading);    

    //sending outputs to drive controller
    m_driveSubsystem.cartesianMecanumDrive(speedX, speedY, () -> rotationOutput);

    //debug info
    SmartDashboard.putNumber("speedX", speedX.getAsDouble());
    SmartDashboard.putNumber("speedY", speedY.getAsDouble());
    SmartDashboard.putNumber("rotationOutput", rotationOutput);
    SmartDashboard.putNumber("rotationSpeed", driverHeadingAdjustment.getAsDouble());
    SmartDashboard.putNumber("expectedHeading", m_expectedHeading);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

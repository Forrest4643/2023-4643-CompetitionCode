// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Sensors;


public class cartesianMecanumDrive extends CommandBase {

  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier speedX, speedY, rotationSpeed;
  private PIDController driveHeadingController = new PIDController(0.003, 0, 0.0031);
  private Sensors m_sensors;

  private double m_expectedHeading;


  /** Creates a new cartesianMecanumDrive. */
  public cartesianMecanumDrive(DriveSubsystem m_driveSubsystem, Sensors m_sensors, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier rotationSpeed) {
    this.m_driveSubsystem = m_driveSubsystem;
    this.m_sensors = m_sensors;
    this.speedX = speedX;
    this.speedY = speedY;
    this.rotationSpeed = rotationSpeed;

    m_expectedHeading = 0;

    driveHeadingController.enableContinuousInput(0, 360);
    addRequirements(m_driveSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_expectedHeading = m_expectedHeading + rotationSpeed.getAsDouble();    

    double rotationOutput = driveHeadingController.calculate(m_sensors.navXYaw(), m_expectedHeading);    

    m_driveSubsystem.cartesianMecanumDrive(speedX, speedY, () -> rotationOutput);

    SmartDashboard.putNumber("speedX", speedX.getAsDouble());
    SmartDashboard.putNumber("speedY", speedY.getAsDouble());
    SmartDashboard.putNumber("rotationSpeed", rotationSpeed.getAsDouble());
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

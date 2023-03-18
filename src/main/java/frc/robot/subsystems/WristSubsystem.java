// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.wristConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class WristSubsystem extends ProfiledPIDSubsystem implements Loggable {

  private static double m_kS = 0;
  private static double m_kG = 0;
  private static double m_kV = 3.63;
  private static double m_kA = 0.07;

  private static double m_kP = 0;
  private static double m_kI = 0;
  private static double m_kD = 0;

  private Sensors m_sensors;

  public final CANSparkMax m_wristMotor = new CANSparkMax(wristConstants.kWristID, MotorType.kBrushless);

  public final SparkMaxAbsoluteEncoder m_wristEncoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

  public ArmFeedforward m_wristFF = new ArmFeedforward(m_kS, m_kG, m_kV, m_kA);

  /** Creates a new wristSubsystem. */
  public WristSubsystem(Sensors m_Sensors) {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            m_kP,
            m_kI,
            m_kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(wristConstants.kMaxVelocityMeters, wristConstants.kMaxAccelMeter)));

            this.m_sensors = m_Sensors;
            m_wristEncoder.setPositionConversionFactor(360);
            m_wristEncoder.setZeroOffset(60);

            getController().setGoal(-80);
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("wristPosition:", m_wristEncoder.getPosition());
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    m_wristMotor.setVoltage(output + m_wristFF.calculate(setpoint.position, setpoint.velocity));
  }

  public boolean atSetpoint() {
    return getController().atSetpoint();
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return Units.degreesToRadians(m_wristEncoder.getPosition());
  }
  
  public void unStow() {
    getController().setGoal(80);
  }

  public void matchStow() {
    getController().setGoal(80);
  }
}

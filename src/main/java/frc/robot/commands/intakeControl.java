// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import com.ctre.phoenix.schedulers.SequentialScheduler;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.Constants.driveConstants;
// import frc.robot.Constants.wristConstants;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.TelescopingSubsystem;
// import frc.robot.subsystems.WristSubsystem;
// import frc.robot.subsystems.mandibleSubsystem;

// public class intakeControl extends CommandBase {

//   private WristSubsystem m_wristSubsystem;
//   private ArmSubsystem m_armSubsystem;
//   private TelescopingSubsystem m_telescopingSubsystem;
//   private mandibleSubsystem m_mandibleSubsystem;

//   private DoubleSupplier m_wristAdjust;

//   private double m_expectedWristPosition;

//   private SlewRateLimiter wristSlewRateLimiter = new SlewRateLimiter(1, -50, 0);


//   /** Creates a new intakeControl. */
//   public intakeControl(WristSubsystem m_WristSubsystem, ArmSubsystem m_ArmSubsystem, 
//   TelescopingSubsystem m_TelescopingSubsystem, mandibleSubsystem m_MandibleSubsystem, DoubleSupplier m_WristAdjust) {

//     this.m_wristSubsystem = m_WristSubsystem;
//     this.m_armSubsystem = m_ArmSubsystem;
//     this.m_telescopingSubsystem = m_TelescopingSubsystem;
//     this.m_mandibleSubsystem = m_MandibleSubsystem;

//     this.m_wristAdjust = m_WristAdjust;

//     addRequirements(m_wristSubsystem, m_armSubsystem, m_telescopingSubsystem, m_mandibleSubsystem);

//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {    
//     m_expectedWristPosition = m_wristSubsystem.getWristPosition();
   
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     double owa = -m_wristAdjust.getAsDouble();
//     double owaOUT;
//     // maintaining minus sign
//     if (owa < 0) {
//       owaOUT = -MathUtil.applyDeadband(
//           Math.sin(
//               wristSlewRateLimiter.calculate(Math.abs(owa))),
//           driveConstants.kInputDeadband);
//     } else {
//       owaOUT = MathUtil.applyDeadband(
//           Math.sin(
//               wristSlewRateLimiter.calculate(Math.abs(owa))),
//           driveConstants.kInputDeadband);
//     }

//     // Takes input from the operator and adjusts the wrists setpoint
//     m_expectedWristPosition = MathUtil.clamp(m_expectedWristPosition + (owaOUT * wristConstants.kWristSensitivity), 
//     wristConstants.kWristMinPositionDEG, wristConstants.kWristMaxPositionDEG);

//     SmartDashboard.putNumber("expectedWristPosition", m_expectedWristPosition);
    
//     m_wristSubsystem.setWristReference(m_expectedWristPosition);

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_wristSubsystem.matchStow();
//     m_telescopingSubsystem.matchStow();
//     m_mandibleSubsystem.intakeHold();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }

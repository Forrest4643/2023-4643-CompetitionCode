//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

public class StickDrive extends CommandBase {

  private final DriveSubsystem m_driveSubsystem;
  private final TurretPIDSubsystem m_turretsubsystem;
  private double m_Speed;
  private double m_turnRate;
  private final XboxController m_driveController;

  public StickDrive(DriveSubsystem m_driveSubsystem, XboxController m_driveController, TurretPIDSubsystem m_turretsubsystem) {

    this.m_driveSubsystem = m_driveSubsystem;
    this.m_driveController = m_driveController;
    this.m_turretsubsystem = m_turretsubsystem;

    addRequirements(m_driveSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("StickDrive Start!");

  }

  // This runs once per scheduler run
  @Override
  public void execute() {
    m_Speed = m_driveController.getRawAxis(5) - m_driveController.getRawAxis(4);
    m_turnRate = m_driveController.getRawAxis(0);

    m_driveSubsystem.setDrive(m_Speed, m_turnRate);

  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setDrive(0, 0);
    System.out.println("StickDrive Ended!");
  }

}

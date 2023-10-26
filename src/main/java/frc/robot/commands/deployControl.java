// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.armConstants;
import frc.robot.Constants.telescopingConstant;
import frc.robot.Constants.wristConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopingSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class deployControl extends CommandBase{
  private ArmSubsystem m_armSubsystem;
  private WristSubsystem m_wristSubsystem;
  private TelescopingSubsystem m_telescopingSubsystem;
  private XboxController m_operateController;

  private String kLow = "LOW";

  private String kMid = "MID";

  private String kHigh = "HIGH";

  private String kConeHighErr = "ERR";


  private int m_deployHeight = 0;

  private String m_deployStatus = kLow;

  private String activeGamepiece = "CUBE";

  private boolean m_coneDeploy = false;

  private boolean m_manualWristControl = false; 


  /** Creates a new ConeDeploy. */
  public deployControl(ArmSubsystem m_ArmSubsystem, WristSubsystem m_WristSubsystem, TelescopingSubsystem m_TelescopingSubsystem, XboxController m_OperateController) {
    this.m_armSubsystem = m_ArmSubsystem;
    this.m_wristSubsystem = m_WristSubsystem;
    this.m_operateController = m_OperateController;
    this.m_telescopingSubsystem = m_TelescopingSubsystem;

    addRequirements(m_armSubsystem, m_wristSubsystem, m_telescopingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_deployHeight = 0;
    m_coneDeploy = false;
    m_manualWristControl = false;
    m_deployStatus = kLow;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_operateController.getAButtonPressed()) {
      m_deployHeight = MathUtil.clamp(m_deployHeight - 1, 0, 2);
    }

    if (m_operateController.getYButtonPressed()) {
      m_deployHeight = MathUtil.clamp(m_deployHeight + 1, 0, 2);
    }

    if (m_operateController.getXButtonPressed()) {
      m_coneDeploy = false;
    } else if (m_operateController.getBButtonPressed()) {
      m_coneDeploy = true;
    }

    if(m_coneDeploy == true) {
      activeGamepiece = "CONE";
    } else {
      activeGamepiece = "CUBE";
    }

      deploySelect(m_deployHeight, m_coneDeploy);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void deploySelect(int deployHeight, boolean coneDeploy) {
    if (coneDeploy == false) {
      switch(deployHeight) {
        case 0:
        m_armSubsystem.setArmReferenceDEG(armConstants.kScoreLowCubePos);
        m_wristSubsystem.setWristReference(wristConstants.kScoreLowCubePos);
        m_telescopingSubsystem.setTelescopingReference(telescopingConstant.kScoreLowCubePos);
        m_deployStatus = kLow;
        break;
        case 1:
        m_armSubsystem.setArmReferenceDEG(armConstants.kScoreMidCubePos);        
        m_wristSubsystem.setWristReference(wristConstants.kScoreMidCubePos);
        m_telescopingSubsystem.setTelescopingReference(telescopingConstant.kScoreMidCubePos);
        m_deployStatus = kMid;
        break;
        case 2:
        m_armSubsystem.setArmReferenceDEG(armConstants.kScoreHighCubePos);        
        m_wristSubsystem.setWristReference(wristConstants.kScoreHighCubePos);
        m_telescopingSubsystem.setTelescopingReference(telescopingConstant.kScoreHighCubePos);
        m_deployStatus = kHigh;
        break;
      }
    } else if (coneDeploy = true) {
      switch(deployHeight) {
        case 0:
        m_armSubsystem.setArmReferenceDEG(armConstants.kScoreLowConePos);
        m_wristSubsystem.setWristReference(wristConstants.kScoreLowConePos);
        m_telescopingSubsystem.setTelescopingReference(telescopingConstant.kScoreLowConePos);
        m_deployStatus = kLow;
        break;
        case 1:
        m_armSubsystem.setArmReferenceDEG(armConstants.kScoreMidConePos);
        m_wristSubsystem.setWristReference(wristConstants.kScoreMidConePos);
        m_telescopingSubsystem.setTelescopingReference(telescopingConstant.kScoreMidConePos);
        m_deployStatus = kMid;
        break;
        case 2:
        m_deployStatus = kConeHighErr;
        break;
    } }

  }
}

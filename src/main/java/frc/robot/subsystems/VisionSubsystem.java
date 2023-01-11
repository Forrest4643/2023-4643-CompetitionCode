//Programmed by Forrest Lasell during the 2022 FRC season for team 4643, Butte Built Bots

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionSystem;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {

    
    PhotonCamera camera = new PhotonCamera("photonvision");

    
            
    double m_targetYaw;

    double m_targetDistanceIN;

    double m_targetDistanceINRAW;

    boolean m_hasTargets;

    public VisionSubsystem() {
        camera.setLED(VisionLEDMode.kOn);
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();

        SmartDashboard.putBoolean("result.hasTargets", result.hasTargets());

        m_hasTargets = result.hasTargets();

        if (result.hasTargets()) {
            m_targetYaw = result.getBestTarget().getYaw();
            m_targetDistanceINRAW = Units.metersToInches(PhotonUtils.calculateDistanceToTargetMeters(
                    VisionConstants.cameraHeightMETERS,
                    VisionConstants.targetHeightMETERS,
                    VisionConstants.cameraAngleRAD,
                    Units.degreesToRadians(result.getBestTarget().getPitch())));
            SmartDashboard.putBoolean("hasTargets", result.hasTargets());
            SmartDashboard.putNumber("targetDistanceIN", getTargetDistanceIN());

            m_targetDistanceIN = m_targetDistanceINRAW;

        // VisionConstants.distC +
        //            (VisionConstants.distB * m_targetDistanceINRAW)
        //             + (Math.pow(m_targetDistanceINRAW, 2) * VisionConstants.distA); 
        } else {
            m_targetYaw = 0;
            m_targetDistanceIN = 0;
            m_targetDistanceINRAW = 0;
        }

    }

    public boolean hasTargets() {
        return m_hasTargets;
    }

    public double getTargetYaw() {
        return m_targetYaw;

    }

    public double getTargetDistanceIN() {
        return m_targetDistanceIN;
    }

    public void LEDon() {

        camera.setLED(VisionLEDMode.kOn);
    }

    public void LEDoff() {
        camera.setLED(VisionLEDMode.kOff);

    }
}

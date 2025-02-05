package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.visionConstants;
import frc.robot.Constants.robot;

public class VisionSubsystem extends SubsystemBase{
    private PhotonCamera m_camera;
    private double m_yaw;
    private double m_pitch;
    private double m_area;
    private double m_skew;
    private double m_targetRange;
    private double m_vrotation;
    private double m_xvspeed;

    public VisionSubsystem() {
     m_camera = new PhotonCamera(visionConstants.kCameraName);
    }

    public void updateStuff(){
        var result = m_camera.getLatestResult();
        // boolean hasTargets = result.hasTargets();
        // List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = result.getBestTarget();

        if(target != null){
            System.out.println("Found target " + target);
            m_yaw = target.getYaw();
            m_pitch = target.getPitch();
            m_area = target.getArea();
            m_skew = target.getSkew();
            m_targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                visionConstants.kCameraHeightMeters,
                visionConstants.kTargetHeightMeters,
                Units.degreesToRadians(-30),
                Units.degreesToRadians(m_pitch));
            m_xvspeed = (1.25 - m_targetRange)*0.07;
            m_vrotation = -1.0*m_yaw*0.07;

            if(target == null){
                m_yaw=0; //TODO FIX
            }
        }
    }

    @Override
    public void periodic() {
        updateStuff();
        SmartDashboard.putNumber("range", m_targetRange);
        SmartDashboard.putNumber("yaw", m_yaw);
        SmartDashboard.putNumber("pitch", m_pitch);
        SmartDashboard.putNumber("area", m_area);
        SmartDashboard.putNumber("skew", m_skew);
    }

    public double getxvspeed(){
        return m_xvspeed;   
    }
    public double getvrotation(){
        return m_vrotation;   
    }

}

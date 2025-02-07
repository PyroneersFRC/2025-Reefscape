package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
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
    private double m_tagID;
    private AprilTagFieldLayout m_aprilTagFieldLayout;
    private Transform3d m_cameraToRobot;
    private double m_distanceToTarget;
    private Pose3d m_targetPose3d;
    private Pose3d m_robotPose;

    public VisionSubsystem() {
     m_camera = new PhotonCamera(visionConstants.kCameraName);
    }

    public void updateStuff(){
        var result = m_camera.getLatestResult();
        // boolean hasTargets = result.hasTargets();
        // List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = result.getBestTarget();

        if(target != null){
            //m_aprilTagFieldLayout = AprilTagFieldLayout.loadField();
            m_tagID = target.getFiducialId();
            m_yaw = target.getYaw();
            m_pitch = target.getPitch();
            m_area = target.getArea();
            m_skew = target.getSkew();
            m_targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                visionConstants.kCameraHeightMeters,
                visionConstants.kTargetHeightMeters,
                Units.degreesToRadians(-30),
                Units.degreesToRadians(m_pitch));
            m_cameraToRobot = visionConstants.cameraToRobot;
            m_targetPose3d = new Pose3d(0, 0, 0,null);
            m_robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),m_targetPose3d, m_cameraToRobot);
}           
            m_distanceToTarget = PhotonUtils.getDistanceToPose(m_robotPose.toPose2d(),m_targetPose3d.toPose2d());
            m_xvspeed=0;
            m_vrotation=0;
        }
    

    @Override
    public void periodic() {
        updateStuff();
        SmartDashboard.putNumber("range", m_targetRange);
        SmartDashboard.putNumber("yaw", m_yaw);
        SmartDashboard.putNumber("pitch", m_pitch);
        SmartDashboard.putNumber("pitch", m_pitch);
        SmartDashboard.putNumber("distance", m_distanceToTarget);

    }

    public double getxvspeed(){
        return m_xvspeed;   
    }
    public double getvrotation(){
        return m_vrotation;   
    }


}
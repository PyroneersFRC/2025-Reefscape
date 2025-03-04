package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.visionConstants;

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
    private Pose2d tagPose2d;
    private Pose2d robotPose2d;

    public VisionSubsystem() {
     m_camera = new PhotonCamera(visionConstants.kCameraName);
     robotPose2d = Pose2d.kZero; //TODO not null
     tagPose2d = new Pose2d(1, 0, Rotation2d.kZero);
    }

    public void updateStuff(){
        var result = m_camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();

        if(target != null){
            //m_aprilTagFieldLayout = AprilTagFieldLayout.load
            m_tagID = target.getFiducialId();
            m_yaw = target.getYaw();
            m_pitch = target.getPitch();
            m_area = target.getArea();
            m_skew = target.getSkew();
            m_targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                visionConstants.kCameraHeightMeters,
                visionConstants.kTargetHeightMeters,
                Units.degreesToRadians(-20),
                Units.degreesToRadians(m_pitch));
            m_cameraToRobot = visionConstants.cameraToRobot;
            m_targetPose3d = new Pose3d(1, 0, 0,new Rotation3d());
            Transform3d cameraToTarget = target.getBestCameraToTarget();

            m_robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, m_targetPose3d, m_cameraToRobot);
            m_distanceToTarget = PhotonUtils.getDistanceToPose(m_robotPose.toPose2d(),m_targetPose3d.toPose2d());
            robotPose2d = m_robotPose.toPose2d();
            tagPose2d = m_targetPose3d.toPose2d();
        }
    }
    

    @Override
    public void periodic() {
        // updateStuff();
        SmartDashboard.putString("Vision/robotPoseTag", robotPose2d.toString());
        SmartDashboard.putString("Vision/tagPose", tagPose2d.toString());
    }

    public Pose2d getTargetPose(){
        return tagPose2d;
    }
    public Pose2d getRobotPoseTag(){
        return robotPose2d;
    }


}
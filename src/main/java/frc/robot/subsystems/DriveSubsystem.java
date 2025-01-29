package frc.robot.subsystems;

import java.time.Period;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANids;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.robot;


public class DriveSubsystem extends SubsystemBase {
    private final SwerveModule m_frontLeft = new SwerveModule(CANids.kFrontLeftDrivingCanId,CANids.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset);
    private final SwerveModule m_frontRight = new SwerveModule(CANids.kFrontRightDrivingCanId,CANids.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset);
    private final SwerveModule m_RearLeft = new SwerveModule(CANids.kRearLeftDrivingCanId,CANids.kRearLeftTurningCanId, DriveConstants.kBackLeftChassisAngularOffset);
    private final SwerveModule m_RearRight = new SwerveModule(CANids.kRearRightDrivingCanId,CANids.kRearRightTurningCanId, DriveConstants.kBackRightChassisAngularOffset);


    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);   // TODO we put random value
    private final SwerveDriveKinematics m_driveKinematics = robot.kDriveKinematics;
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_driveKinematics,
          Rotation2d.fromDegrees(m_gyro.getAngle()),new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_RearLeft.getPosition(),
            m_RearRight.getPosition()
          });

    //private final Pose2d m_Pose2d = new Pose2d(getPose2d().getTranslation(),getPose2d().getRotation());

    
    public DriveSubsystem() {
        RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
        // Configure AutoBuilder
        AutoBuilder.configure(
            this::getPose2d, 
            this::resetPose2d, 
            this::getCurrentSpeeds,
            this::driveRobotRelative, 
            new PPHolonomicDriveController(
            robot.kPIDDrive,
            robot.kPIDTurning
            ), config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
             } , this);


    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

}
    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative){
        //System.out.println("xSpeed: " + xSpeed + " ySpeed " + ySpeed + " rotation " + rotation);

        double xSpeedDelivered = 0.3 * xSpeed * robot.kPhysicalMaxSpeedMetersPerSecond;
        double ySpeedDelivered = 0.3 * ySpeed * robot.kPhysicalMaxSpeedMetersPerSecond;
        double rotationDelivered = 0.3 * rotation * robot.kPhysicalMaxAngularSpeedRadiansPerSecond;

        var swerveModuleStates = robot.kDriveKinematics.toSwerveModuleStates(
            fieldRelative 
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered, Rotation2d.fromDegrees(m_gyro.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered)
        );

        this.setModuleStates(swerveModuleStates);
    }

    @Override
    public void periodic(){
        m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_RearLeft.getPosition(),
          m_RearRight.getPosition()
        });
        System.out.println(m_RearLeft.getPosition());
    }
    public void zeroHeading(){
        m_gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }
    public Rotation2d getRotation2dZero(){
        return Rotation2d.fromDegrees(0);
    }
    public Pose2d getPose2d(){
        return m_odometry.getPoseMeters();
    }
    public void resetPose2d(Pose2d pose2d){
        m_odometry.resetPose(pose2d);
    }
    public ChassisSpeeds getCurrentSpeeds(){
        return m_driveKinematics.toChassisSpeeds(new SwerveModuleState[]{m_frontLeft.getState(),m_frontRight.getState(),m_RearLeft.getState(),m_RearRight.getState()});
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = m_driveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
      }

    public void stopModules() {
        m_RearLeft.stop();
        m_RearRight.stop();
        m_frontLeft.stop();
        m_frontRight.stop();

    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, robot.kPhysicalMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_RearLeft.setDesiredState(desiredStates[2]);
        m_RearRight.setDesiredState(desiredStates[3]);
        
    }
}

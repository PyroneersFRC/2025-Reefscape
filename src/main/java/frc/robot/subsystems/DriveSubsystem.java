package frc.robot.subsystems;

import java.time.Period;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANids;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.robot;


public class DriveSubsystem extends SubsystemBase {
    private final SwerveModule m_frontLeft = new SwerveModule(CANids.kFrontLeftDrivingCanId,CANids.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset);
    private final SwerveModule m_frontRight = new SwerveModule(CANids.kFrontRightDrivingCanId,CANids.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset);
    private final SwerveModule m_RearLeft = new SwerveModule(CANids.kRearLeftDrivingCanId,CANids.kRearLeftTurningCanId, DriveConstants.kBackLeftChassisAngularOffset);
    private final SwerveModule m_RearRight = new SwerveModule(CANids.kRearRightDrivingCanId,CANids.kRearRightTurningCanId, DriveConstants.kBackRightChassisAngularOffset);

    //private final SwerveModulePosition m_ModulePositions = new SwerveModulePosition(, getRotation2d());


    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);   // TODO we put random value
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(robot.kDriveKinematics,
          Rotation2d.fromDegrees(m_gyro.getAngle()),new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_RearLeft.getPosition(),
            m_RearRight.getPosition()
          });

    // private final Pose2d m_pose = new Pose2d(null, getRotation2d());
    // private final Pose2d m_poseZero = new Pose2d(0,0, getRotation2dZero());
    
    public DriveSubsystem() {
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                zeroHeading();
            }catch (Exception e){}
        }).start();
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
    public void resetPose2d(){
        m_odometry.resetPose(getPose2d());
    }
    // public ChassisSpeeds getCurrentSpeeds(){
    //     return robot.kDriveKinematics.toChassisSpeeds();
    // }
    //TODO MESA STIN PARENTHESI

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

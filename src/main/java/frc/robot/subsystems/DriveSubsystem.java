
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANids;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.robot;
import frc.robot.Constants.xboxConstants;


public class DriveSubsystem extends SubsystemBase {
    private final SwerveModule m_frontLeft = new SwerveModule(CANids.kFrontLeftDrivingCanId,CANids.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset);
    private final SwerveModule m_frontRight = new SwerveModule(CANids.kFrontRightDrivingCanId,CANids.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset);
    private final SwerveModule m_rearLeft = new SwerveModule(CANids.kRearLeftDrivingCanId,CANids.kRearLeftTurningCanId, DriveConstants.kBackLeftChassisAngularOffset);
    private final SwerveModule m_rearRight = new SwerveModule(CANids.kRearRightDrivingCanId,CANids.kRearRightTurningCanId, DriveConstants.kBackRightChassisAngularOffset);

    SlewRateLimiter xlimiter = new SlewRateLimiter(robot.kTeleDriveAccelerationUnitsPerSecond);
    SlewRateLimiter ylimiter = new SlewRateLimiter(robot.kTeleDriveAccelerationUnitsPerSecond);
    SlewRateLimiter rotationlimiter = new SlewRateLimiter(40);

    private final Rotation2d m_rotation = new Rotation2d(Math.PI/6);
    // private final VisionSubsystem m_vision = new VisionSubsystem();


    private static final double k1 = 0.5;
    private static final double k2 = 0.3;
    

    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);   // TODO we put random value
    private final SwerveDriveKinematics m_driveKinematics = robot.kDriveKinematics;
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_driveKinematics,
          Rotation2d.fromDegrees(m_gyro.getAngle()),new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

    
    public DriveSubsystem() {
        RobotConfig config;
        try {
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
    public Command goToPose() {
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    0.3*robot.kPhysicalMaxSpeedMetersPerSecond,
                    0.3*robot.kTeleDriveAccelerationUnitsPerSecond)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(robot.kDriveKinematics);
    
        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                getPose2d(),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(getPose2d().getX()+1, getPose2d().getY(),getRotation2d().plus(new Rotation2d(Math.PI/2))),
                config);
    
        var thetaController =
            new ProfiledPIDController(
                0.4, 0, 0.2, new TrapezoidProfile.Constraints(0.5, 0.5));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                this::getPose2d, // Functional interface to feed supplier
                robot.kDriveKinematics,
    
                // Position controllers
                new PIDController(3, 0.35, 0),
                new PIDController(3, 0.35, 0),
                thetaController,
                this::setModuleStates,
                this);
    
        // Reset odometry to the initial pose of the trajectory, run path following
        // command, then stop at the end.
        return Commands.sequence(
            new InstantCommand(() -> this.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> this.drive(0, 0, 0, true)));
    }
    
        

    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative){
        xSpeed = xlimiter.calculate(xSpeed);
        ySpeed = ylimiter.calculate(ySpeed);
        rotation = rotationlimiter.calculate(rotation);
       
        var swerveModuleStates = robot.kDriveKinematics.toSwerveModuleStates(
            fieldRelative 
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, Rotation2d.fromDegrees(-m_gyro.getAngle()))
                : new ChassisSpeeds(xSpeed, ySpeed, rotation)
        );

        this.setModuleStates(swerveModuleStates);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("DriveSubsystem/Gyro", m_gyro.getAngle());
        SmartDashboard.putString("DriveSubsystem/pose", getPose2d().toString());

        


        m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()), new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
    }


    public void zeroHeading(){
        m_gyro.reset();
        m_odometry.resetPose(getPose2d());
    }

    public Command resetGyro(){
        return this.runOnce(this::zeroHeading);
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


    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            pose);
    }

    public ChassisSpeeds getCurrentSpeeds(){
        return m_driveKinematics.toChassisSpeeds(new SwerveModuleState[]{m_frontLeft.getState(),m_frontRight.getState(),m_rearLeft.getState(),m_rearRight.getState()});
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = m_driveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
      }

    public void stopModules() {
        m_rearLeft.stop();
        m_rearRight.stop();
        m_frontLeft.stop();
        m_frontRight.stop();

    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, robot.kPhysicalMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
        
    }

    public Command driveWithJoystickCmd(CommandXboxController xboxController, double precision){
        return this.run(
                () ->
                    this.drive(
                        - robot.kPhysicalMaxSpeedMetersPerSecond * precision * MathUtil.applyDeadband(
                            xboxController.getLeftY(), xboxConstants.kDeadband),
                        - robot.kPhysicalMaxSpeedMetersPerSecond * precision * MathUtil.applyDeadband(
                            xboxController.getLeftX(), xboxConstants.kDeadband),
                        - robot.kPhysicalMaxAngularSpeedRadiansPerSecond * precision * MathUtil.applyDeadband(
                            xboxController.getRightX(), xboxConstants.kDeadband),
                        true)
                );
    }


    private final SysIdRoutine m_sysIdRoutine = 
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.5),Volts.of(2.5),5),
            new SysIdRoutine.Mechanism(
                voltage -> {
                    double volt = voltage.in(Volts);
                    this.drive(0, 0, 1, true);
                    m_rearLeft.setDriveVoltage(volt); 
                    m_frontLeft.setDriveVoltage(volt);
                    m_rearRight.setDriveVoltage(volt);
                    m_frontRight.setDriveVoltage(volt);
                    System.out.println(volt);
                },
                null,
                this
            )
        );

    /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
       return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }


}
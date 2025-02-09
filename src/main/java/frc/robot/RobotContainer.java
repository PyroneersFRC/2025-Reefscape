package frc.robot;

import java.nio.file.FileSystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.autoAlignCmd;
import frc.robot.Constants.CANids;
import frc.robot.Constants.xboxConstants;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {


  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final CommandXboxController m_driverController = new CommandXboxController(Constants.xboxConstants.kDriverControllerPort);
  //private final Command m_autoAlign = new autoAlignCmd(m_driveSubsystem,m_visionSubsystem);
  private final ElevatorSubsystem  m_elevatorSubsystem = new ElevatorSubsystem(CANids.kRightElevatorCanId, CANids.kLeftElevatorCanId);


    public RobotContainer() {
      configureButtonBindings();
            m_driveSubsystem.setDefaultCommand(Commands.run(
            () ->
                m_driveSubsystem.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), xboxConstants.kDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), xboxConstants.kDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), xboxConstants.kDeadband),
                      true),
            m_driveSubsystem));
    }


  private void configureButtonBindings() {
      m_driverController.b().onTrue(m_driveSubsystem.resetGyro());
      //m_driverController.a().whileTrue(m_driveSubsystem.goToPose());
      m_driverController.leftBumper().onTrue(m_elevatorSubsystem.decrease());
      m_driverController.rightBumper().onTrue(m_elevatorSubsystem.accelerate());
      m_driverController.y().onTrue(m_elevatorSubsystem.runElevator());
      m_driverController.x().onTrue(m_elevatorSubsystem.stopElevator());
  }


  public Command getAutonomousCommand() {
    return new PathPlannerAuto("auto1");
  }
}
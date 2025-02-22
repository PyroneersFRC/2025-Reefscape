package frc.robot;

import java.nio.file.FileSystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OutakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.autoAlignCmd;
import frc.robot.Constants.CANids;
import frc.robot.Constants.OutakeConstants;
import frc.robot.Constants.xboxConstants;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(xboxConstants.kDriverControllerPort);
  private final CommandXboxController m_OperatorController = new CommandXboxController(xboxConstants.kOperatorControllerPort);


  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final OutakeSubsystem m_outakeSubsystem = new OutakeSubsystem(CANids.KOutakeCanId);

    public RobotContainer() {
      configureButtonBindings();
            m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveWithJoystickCmd(m_driverController));
     }


  private void configureButtonBindings() {
      m_driverController.b().onTrue(m_driveSubsystem.resetGyro());
      m_driverController.a().onTrue(m_driveSubsystem.goToPose());

      // m_driverController.povDown().onTrue(m_elevatorSubsystem.runElevatorCmd(0));
      // m_driverController.povRight().onTrue(m_elevatorSubsystem.runElevatorCmd(1));
      // m_driverController.povLeft().onTrue(m_elevatorSubsystem.runElevatorCmd(2));
      // m_driverController.povUp().onTrue(m_elevatorSubsystem.runElevatorCmd(3));
      m_driverController.rightBumper().onTrue(m_elevatorSubsystem.setLevel(1));
      m_driverController.leftBumper().onTrue(m_elevatorSubsystem.setLevel(0));
      m_driverController.leftTrigger().onTrue(m_elevatorSubsystem.setLevel(2));
      m_driverController.y().onTrue(m_elevatorSubsystem.runElevator());
      m_driverController.x().onTrue(m_elevatorSubsystem.stopCmd());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("auto1");
  }
}
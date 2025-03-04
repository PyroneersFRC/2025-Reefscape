// package frc.robot;

// import com.fasterxml.jackson.databind.util.internal.PrivateMaxEntriesMap;

// import frc.robot.subsystems.OutakeSubsystem;

// public class AutoRoutines {
//     private final OutakeSubsystem m_outakeSubsystem;

//     public AutoRoutines(OutakeSubsystem outakeSubsystem){
//         m_outakeSubsystem = outakeSubsystem;
//     }

//     public static Command foo(){
//         return m_outakeSubsystem.outakeCmd().andThen(new WaitCommand(0.7)).andThen(m_outakeSubsystem.zeroCmd().andThen(m_elevatorSubsystem.setLevel(Level.Level0))));
//     }

// }

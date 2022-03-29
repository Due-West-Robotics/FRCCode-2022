package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoDoNothing extends SequentialCommandGroup{
    
    public AutoDoNothing() {
        addCommands(
            new WaitCommand(14));
      }
}

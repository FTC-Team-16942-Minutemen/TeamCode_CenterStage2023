package org.firstinspires.ftc.teamcode.autons;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

public class BlueRightAuton {

    public BlueRightAuton()
    {
        // inject subsystems and store them here
    }

    public Command generate()
    {
           return new SequentialCommandGroup(
                   new WaitCommand(250)

                //write your auton here as a huge command sequential command group (i.e. series of commands)
           );
    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoTurnASquare extends SequentialCommandGroup {

  //Creates a new AutoTurnASquare.
  public AutoTurnASquare() {

    /**
    // Add your commands in the super() call, e.g. super(new FooCommand(), new BarCommand());
    super( new AutoDriveStraightTime(0.45, 2.0).andThen(new WaitCommand(0.4)), // go forward
        new AutoTurnToAngle(-90.0, 0.40).andThen(new WaitCommand(0.4)),       // turn right
        new AutoDriveStraightTime(0.45, 2.0).andThen(new WaitCommand(0.4)),   // go forward
        new AutoTurnToAngle(-90.0, 0.40).andThen(new WaitCommand(0.4)),       // turn right
        new AutoDriveStraightTime(0.45, 2.0).andThen(new WaitCommand(0.4)),   // go forward
        new AutoTurnToAngle(-90.0, 0.40).andThen(new WaitCommand(0.4)),       // turn right
        new AutoDriveStraightTime(0.45, 2.0)    // go forward to starting point
    );
    */

    // alternate way to do a square
    for (int i = 0; i < 4; i++) {
      addCommands(new AutoDriveStraightTime(0.45, 2.0).andThen(new WaitCommand(0.4)));  // repeat 4 times
      addCommands(new AutoSpinToAngle(-90.0, 0.40).andThen(new WaitCommand(0.4)));
    }
    
  }
}

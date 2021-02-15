/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoDriveStraightTime;
import frc.robot.commands.AutoSpinToAngle;
import frc.robot.commands.AutoTurnToAngle;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoRunChallenge3 extends SequentialCommandGroup {
  /**
   * Creates a new AutoRunChallenge3.
   */
  public AutoRunChallenge3(DriveTrain driveTrain) {

    // Add your commands in the super() call, e.g. super(new FooCommand(), new BarCommand());
    /* super( new AutoSpinToAngle(45.0, 0.50).andThen(new WaitCommand(0.4)), // turn to 45 deg angle
      new AutoDriveStraightTime(0.40, 3.0).andThen(new WaitCommand(0.4)), // go forward to clear row
      new AutoTurnToAngle(0.0, 0.50).andThen(new WaitCommand(0.4)),       // turn back to 0 (west)
      new AutoDriveStraightTime(0.40, 5.0).andThen(new WaitCommand(0.4)),  // go forward to first turn
      new AutoTurnToAngle(-45.0, 0.50).andThen(new WaitCommand(0.4)),      // turn to -45 deg angle
      new AutoDriveStraightTime(0.40, 4.0).andThen(new WaitCommand(0.4)),  // go forward to clear row
      new AutoTurnToAngle(0.0, 0.50).andThen(new WaitCommand(0.4)),        // back to 0 (west)
      new AutoDriveStraightTime(0.40, 3.0).andThen(new WaitCommand(0.4)),  // go forward to clear block
      new AutoTurnToAngle(90.0, 0.50).andThen(new WaitCommand(0.4)),       // turn to 90 (north)
      new AutoDriveStraightTime(0.40, 5.0).andThen(new WaitCommand(0.4)),  // go forward to clear last block
      new AutoTurnToAngle(0.0, 0.50).andThen(new WaitCommand(0.4)),       // turn to 0 (west)
      new AutoDriveStraightTime(0.40, 3.0).andThen(new WaitCommand(0.4)),  // go forward to clear last block
      new AutoTurnToAngle(-90.0, 0.50).andThen(new WaitCommand(0.4)),      // turn to -45 deg angle
      new AutoDriveStraightTime(0.40, 3.0).andThen(new WaitCommand(0.4)),  // go forward to clear row
      new AutoTurnToAngle(180.0, 0.50).andThen(new WaitCommand(0.4)),      // turn to 180 deg angle
      new AutoDriveStraightTime(0.40, 6.0).andThen(new WaitCommand(0.4)),  // go back to start
      new InstantCommand(driveTrain::stop, driveTrain)
 */ 

    super (new AutoDriveStraightTime(0.50, 2.0).andThen(new WaitCommand(0.4)), //forward 

    new AutoTurnToAngle(30.0, 0.50).andThen(new WaitCommand(0.4)), //turn down 
    new AutoDriveStraightTime(0.50, 2.0).andThen(new WaitCommand(0.4)), //left
    new AutoTurnToAngle(-30.0, 0.50).andThen(new WaitCommand(0.4)), // turn up 
    new AutoDriveStraightTime(0.50, 2.0).andThen(new WaitCommand(0.4)), // up
    new AutoTurnToAngle(-30.0, 0.50).andThen(new WaitCommand(0.4)), // turn right 
    new AutoDriveStraightTime(0.50, 2.0).andThen(new WaitCommand(0.4)), // forward but up a bit 

    new AutoDriveStraightTime(0.50, 2.0).andThen(new WaitCommand(0.4)), //up  
    new AutoTurnToAngle(45.0, 0.50).andThen(new WaitCommand(0.4)), // turn left 
    new AutoSpinToAngle(30.0, 0.50).andThen(new WaitCommand(0.4)), // turn down  
    new AutoDriveStraightTime(0.50, 2.0).andThen(new WaitCommand(0.4)), // go down  

    new AutoTurnToAngle(-30.0, 0.50).andThen(new WaitCommand(0.4)), // turn right a bit 
    new AutoDriveStraightTime(0.50, 2.0).andThen(new WaitCommand(0.4)), // up 
    new AutoTurnToAngle(45.0, 0.50).andThen(new WaitCommand(0.4)), // turn left
    new InstantCommand(driveTrain::stop, driveTrain) //the big red botton 
    ); 
  }
}
 




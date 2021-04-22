/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
//import frc.robot.commands.AutoDriveStraightTime;
import frc.robot.commands.AutoDriveStraightUnits;
import frc.robot.commands.AutoSpinToAnglePID;
import frc.robot.subsystems.DriveTrain;

/**
 * A complex autonomous command that drives forward, and then drives backward.
 */
public class AutoGoSlalomPath extends SequentialCommandGroup {
  
   /**
   * Creates a new SequentialCommandGroup
   *
   * @param driveTrain The DriveTrain subsystem this command will run on
   */
  public AutoGoSlalomPath(DriveTrain driveTrain) {
    
    // positive angles turn right, negative angles turn left
    addCommands(
	    new InstantCommand(() -> driveTrain.stop(), driveTrain),    // make sure stopped
      new AutoDriveStraightUnits(-0.5, 50), //forward DONE
      new AutoSpinToAnglePID(45.0, 0.50), //turn down DONE
      new AutoDriveStraightUnits(-0.50,100), //leftDONE
      new AutoSpinToAnglePID( -45.0, 0.5), //Temp done
      new AutoDriveStraightUnits(-0.50, 200), // up DONE
      new AutoSpinToAnglePID(90.0, 0.40), // turn right DONE
      new AutoDriveStraightUnits(-0.50, 20),
      new AutoSpinToAnglePID(100.0, 0.40), // turn right DONE
      new AutoDriveStraightUnits(-0.50, 20), //up  

      /* Temp
      // forward but up a bit 
      new AutoSpinToAnglePID(45.0, 0.50), // turn left 
      new AutoSpinToAnglePID(30.0, 0.50), // turn down  
      new AutoDriveStraightTime(0.50, 2.0), // go down  
      new AutoSpinToAnglePID(-30.0, 0.50), // turn right a bit 
      new AutoDriveStraightTime(0.50, 2.0), // up 
      new AutoSpinToAnglePID(45.0, 0.50), // turn left */ 
      
      new InstantCommand(driveTrain::stop, driveTrain)       // make sure stopped
    );
  }
}

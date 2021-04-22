/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoDriveStraightTime;
import frc.robot.commands.AutoSpinToAnglePID;
import frc.robot.subsystems.DriveTrain;

/**
 * A complex autonomous command that drives forward, and then drives backward.
 */
public class AutoGoBarrelPath extends SequentialCommandGroup {

  /**
   * Creates a new SequentialCommandGroup
   *
   * @param driveTrain The DriveTrain subsystem this command will run on
   */
  public AutoGoBarrelPath(DriveTrain driveTrain) {

      // positive angles turn right, negative angles turn left
    addCommands(
      new InstantCommand(() -> driveTrain.stop(), driveTrain),    // make sure stopped
      new AutoDriveStraightTime(-0.75, 3.5), //forward DONE
      new AutoSpinToAnglePID(75.0, 0.50), //turn down DONE
      new AutoDriveStraightTime(-0.50, 3.5), //left DONE
      new AutoSpinToAnglePID( 80.0, 0.50), //Temp DONE
      new AutoDriveStraightTime(-0.60, 2.0), // up DONE
      new AutoSpinToAnglePID(80.0, 0.50), // turn right DONE
      new AutoDriveStraightTime(-0.50, 3.5), // ^done, dont do anything else
      new WaitCommand(0.25), 
      new AutoSpinToAnglePID(80.0, 0.50), // turn right DONE

      new AutoDriveStraightTime(-0.75, 3.75), //forward 
      new AutoSpinToAnglePID(-80.0, 0.50), // turn left 
      new AutoDriveStraightTime(-0.50, 3.5), //left DONE
      new AutoSpinToAnglePID(-80.0, 0.50), // turn left 
      new AutoDriveStraightTime(-0.50, 3.5), //left DONE
      new AutoSpinToAnglePID(-80.0, 0.50), // turn left 
      new AutoDriveStraightTime(-0.50, 3.5), //left DONE

      new AutoSpinToAnglePID(-65.0, 0.50), // turn left 
      new AutoDriveStraightTime(-0.75, 3.5), //forward 
      new AutoSpinToAnglePID(-80.0, 0.50), // turn left 
      new AutoDriveStraightTime(-0.50, 4.3), //left DONE
      new AutoSpinToAnglePID(-55.0, 0.50), // turn left 
      new WaitCommand(1.0), 
      new AutoDriveStraightTime(-0.75, 8.52), //forward DONE

      /* Temp
      // forward but up a bit 
      new AutoDriveStraightTime(0.50, 2.0), //up  
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

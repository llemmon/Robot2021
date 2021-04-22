/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoDriveStraightTime;
import frc.robot.subsystems.DriveTrain;

public class AutoGoRedPath extends SequentialCommandGroup {

  public AutoGoRedPath(DriveTrain driveTrain) {

    // pass all commands to super (super is the construtor of this class's parent)
    addCommands(
          new InstantCommand(() -> driveTrain.stop(), driveTrain),
          new AutoDriveStraightTime( 0.45, 2.0),
          new AutoSpinToAnglePID( 30.0, 0.45),
          new AutoDriveStraightTime( 0.45, 2.0),
          new AutoSpinToAnglePID(-40.0, 0.45),
          new InstantCommand(driveTrain::stop, driveTrain)
    ); //Positive is Right , Negative is Leftt

      //super(new InstantCommand(() -> driveTrain.stop(), driveTrain),
      //new ParallelCommandGroup(new AutoDriveStraightTime( 0.45, 2.0), new WaitCommand(3.0)),
      //new ParallelDeadlineGroup(new AutoTurnToAngle(45.0, 0.5), new WaitCommand(3.0)),
      //new AutoDriveStraightTime( 0.45, 2.0),
      //new SequentialCommandGroup(new AutoSpinToAngle( -45.0, 2.0), new WaitCommand(2.0)),
      //new InstantCommand(driveTrain::stop, driveTrain)
  }

}

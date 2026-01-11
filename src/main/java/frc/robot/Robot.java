// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.concurrent.Flow.Publisher;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.PubSubOptions;
import edu.wpi.first.networktables.StructArrayEntry;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  public RobotContainer robotContainer;

  RobotState robotState;

  final NetworkTable table;

  final StructEntry<Pose2d> poseEntry;

  final StructEntry<ChassisSpeeds> chassisEntry;

  final StructArrayEntry<SwerveModuleState> states;

  final StructArrayEntry<SwerveModuleState> real;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {

    robotContainer = new RobotContainer();

    Logger.start();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    table = inst.getTable("blud");

    poseEntry = inst.getStructTopic("/blud/estimatedPose", Pose2d.struct).getEntry(
      RobotState.getInstance().getPose(), 
      PubSubOption.keepDuplicates(true)
    );

    chassisEntry = inst.getStructTopic("/blud/chassisSpeeds", ChassisSpeeds.struct).getEntry(
      robotContainer.drive.chassisSpeeds,
      PubSubOption.keepDuplicates(true)
    );

    real = inst.getStructArrayTopic("/blud/real", SwerveModuleState.struct).getEntry(
      robotContainer.drive.getModuleStates(),
      PubSubOption.keepDuplicates(true)
    );

    states = inst.getStructArrayTopic("/blud/states", SwerveModuleState.struct).getEntry(
      robotContainer.drive.kinematics.toSwerveModuleStates(robotContainer.drive.chassisSpeeds),
      PubSubOption.keepDuplicates(true)
    );

    robotState = RobotState.getInstance();

  }




  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    poseEntry.set(
      RobotState.getInstance().getPose()
    );

    states.set(
      robotContainer.drive.kinematics.toSwerveModuleStates(robotContainer.drive.chassisSpeeds)
    );

    real.set(
      robotContainer.drive.getModuleStates()
    );

    chassisEntry.set(
      robotContainer.drive.chassisSpeeds
    );
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {

    SimulatedArena.getInstance();
    
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {

    
    SimulatedArena.getInstance().simulationPeriodic();
  }
}

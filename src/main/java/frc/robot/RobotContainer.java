// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.module.ModuleIOFullKraken;
import frc.robot.subsystems.drive.module.ModuleIOReal;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Hood.HoodIOSim;
import frc.robot.subsystems.shooter.Turret.Turret;
import frc.robot.subsystems.shooter.Turret.TurretIO;
import frc.robot.subsystems.shooter.Turret.TurretIOMotor;
import frc.robot.subsystems.shooter.Turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.Mode;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  XboxController controller;

  Drive drive;

  Shooter shooter;

  Vision vision;

  SwerveDriveSimulation driveSim;
  final DriveTrainSimulationConfig driveSimConfig = DriveTrainSimulationConfig.Default()
        // Specify gyro type (for realistic gyro drifting and error simulation)
        .withGyro(COTS.ofNav2X())
        // Specify swerve module (for realistic swerve dynamics)
        .withSwerveModule(COTS.ofMark4(
                DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                DCMotor.getKrakenX60(1), // Steer motor is a Falcon 500
                COTS.WHEELS.BLUE_NITRILE_TREAD.cof, // wheel cof
                3)) // L3 Gear ratio
        // Configures the track length and track width (spacing between swerve modules)
        .withTrackLengthTrackWidth(Inches.of(26), Inches.of(28))
        // Configures the bumper size (dimensions of the robot bumper)
        .withBumperSize(Inches.of(30), Inches.of(30))
        .withRobotMass(Pounds.of(100));
        

  public void mapleSimSetup()
  {
    SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
  }
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings


    if (Constants.mode == Mode.REAL)
    {


      this.drive = new Drive(
        new ModuleIOReal(Constants.flDriveID, Constants.flEncoderID, Constants.flTurnID, 0),
        new ModuleIOReal(Constants.frDriveID, Constants.frEncoderID, Constants.frTurnID, 1),
        new ModuleIOReal(Constants.blDriveID, Constants.blEncoderID, Constants.blTurnID, 2),
        new ModuleIOReal(Constants.brDriveID, Constants.brEncoderID, Constants.brTurnID, 3),
        new GyroIOReal(),
        Constants.SimConstants.turnP.get(), Constants.SimConstants.turnI.get(), Constants.SimConstants.turnD.get(),
        Constants.SimConstants.driveP.get(), Constants.SimConstants.driveD.get()
      );

    }
    else
    {

      this.shooter = new Shooter(
        new TurretIOMotor(),
        new HoodIOSim(),
        Constants.SimConstants.turretP.get(), 
        Constants.SimConstants.turretI.get(),
        Constants.SimConstants.turretD.get(),
        0,0,0
      );

      this.driveSim = new SwerveDriveSimulation(
        driveSimConfig,
        RobotState.getInstance().getPose()
      ); 

      this.drive = new Drive(
        
        new ModuleIOSim(driveSim.getModules()[0],0),
        new ModuleIOSim(driveSim.getModules()[1],1),
        new ModuleIOSim(driveSim.getModules()[2],2),
        new ModuleIOSim(driveSim.getModules()[3],3),
        new GyroIOSim(driveSim.getGyroSimulation()),
        Constants.SimConstants.turnP.get(), Constants.SimConstants.turnI.get(), Constants.SimConstants.turnD.get(),
        Constants.SimConstants.driveP.get(), Constants.SimConstants.driveD.get()
      );

      this.shooter = new Shooter(
        new TurretIOSim(),
        new HoodIOSim(),
        Constants.SimConstants.turretP.get(), 
        Constants.SimConstants.turretI.get(),
        Constants.SimConstants.turretD.get(),
        0,0,0
      );

      mapleSimSetup();
    }
    controller = new XboxController(0);

    




    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    drive.setDefaultCommand(
      Drive.driveRobotCentric
      (
        drive, 
        () -> controller.getLeftX(),
        () -> controller.getLeftY(),
        () -> controller.getRightX()
      )
    );

    shooter.setDefaultCommand(
      Shooter.autoAlignCommand
      (
        shooter
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}

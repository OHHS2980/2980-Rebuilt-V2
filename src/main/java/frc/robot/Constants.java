// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static int flDriveID = 20;
  public static int flTurnID = 10;
  public static int flEncoderID = 21;


  public static int frDriveID = 16;
  public static int frTurnID = 14;
  public static int frEncoderID = 23;


  public static int blDriveID = 17;
  public static int blTurnID = 12;
  public static int blEncoderID = 22;


  public static int brDriveID = 18;
  public static int brTurnID = 16;
  public static int brEncoderID = 24;


  public static int turretID = 8;
  public static int turretID2 = 10;
  public static int turretEncoderID = 6;
  public static int turretEncoderID2 = 7;
  



  public enum Mode 
  {
    SIM,
    REAL,
    TEST_DRIVE,
    TEST_TURRET
  }

  public static Mode mode = Mode.REAL;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SimConstants {

    public static LoggedNetworkNumber turnP = new LoggedNetworkNumber("/Tuning/turnP", 0.1);

    public static LoggedNetworkNumber turnI = new LoggedNetworkNumber("/Tuning/turnI", 0);
  
    public static LoggedNetworkNumber  turnD = new LoggedNetworkNumber ("/Tuning/turnD", 0);
  
    public static LoggedNetworkNumber  driveP = new LoggedNetworkNumber("/Tuning/driveP", 2);
  
    public static LoggedNetworkNumber  driveD = new LoggedNetworkNumber("/Tuning/driveD", 0);



    public static LoggedNetworkNumber turretP = new LoggedNetworkNumber("/Tuning/turretP", 0.05);

    public static LoggedNetworkNumber turretI = new LoggedNetworkNumber("/Tuning/turretI", 0);
  
    public static LoggedNetworkNumber  turretD = new LoggedNetworkNumber ("/Tuning/turretD", 1);
    
  }

  public static class FieldConstants {

    public static Pose2d getHub()
    {
      return new Pose2d(4.0132,4.572, null);
    }

    public static double hubHeight()
    {
      return 1.828;
    }
    
  }


  
  public static final double MAX_SPEED = 0;

  public static Translation2d flLocation = new Translation2d(0.3556,0.3302);
  public static Translation2d frLocation = new Translation2d(0.3556,-0.3302);
  public static Translation2d blLocation = new Translation2d(-0.3556,-0.3302);
  public static Translation2d brLocation = new Translation2d(-0.3556,0.3302);

  public static Translation2d[] locations = new Translation2d[] {flLocation, frLocation, blLocation, brLocation};

  //CAN IDs

  public static final int armEncoderID = 0;

  public static int armCanID;

  public static double trackY;

  public static double trackX;

  public static double bumperY;

  public static double bumperX;

  public static double robotMass;

  public static double swerveWheelRadius = 0.0381;

  public static double driveGearRatio = 6;

  public static double turnGearRatio = 6;

  public static double turretLimit = 400;

  public static String limelightName;

  public static String camName;
  
  public static double TURRET_GEAR_RATIO = 100;

}

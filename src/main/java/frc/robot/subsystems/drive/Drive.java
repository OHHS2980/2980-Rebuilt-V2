package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.module.Module;

import frc.robot.subsystems.drive.module.ModuleIO;

public class Drive extends SubsystemBase {

    @AutoLogOutput
    public ChassisSpeeds chassisSpeeds;

    public SwerveDriveKinematics kinematics;

    public SwerveDriveSimulation driveSim;

    public SwerveDriveOdometry odometry;

    public GyroIO gyroIO;

    public Module flModule, frModule, blModule, brModule;

    public Module[] modules = {flModule, frModule, blModule, brModule};

    public ModuleIO[] moduleIOs = new ModuleIO[4];

    public SwerveModuleState[] realModuleStates = new SwerveModuleState[4];

    public SwerveModuleState[] moduleStates = new SwerveModuleState[4];

    public SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];


    @AutoLogOutput
    public Pose2d pose = new Pose2d();

    public Drive(
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      GyroIO gyroIO,
      double kP, double kI, double kD,

      double drive_kP, double drive_kD
    ) {

        flModule = new Module(flModuleIO, kP, kI, kD, drive_kP, drive_kD);
        frModule = new Module(frModuleIO, kP, kI, kD, drive_kP, drive_kD);
        blModule = new Module(blModuleIO, kP, kI, kD, drive_kP, drive_kD);
        brModule = new Module(brModuleIO, kP, kI, kD, drive_kP, drive_kD);

        moduleIOs[0] = flModuleIO;
        moduleIOs[1] = frModuleIO;
        moduleIOs[2] = blModuleIO;
        moduleIOs[3] = brModuleIO;


        this.gyroIO = gyroIO;

        chassisSpeeds = new ChassisSpeeds();

        modules[0] = flModule;
        modules[1] = frModule;
        modules[2] = blModule;
        modules[3] = brModule;

        kinematics = new SwerveDriveKinematics(
            Constants.flLocation,
            Constants.frLocation, 
            Constants.blLocation, 
            Constants.brLocation
        );

        odometry = new SwerveDriveOdometry
        (
            kinematics, 
            gyroIO.getHeading(), 
            new SwerveModulePosition[] 
            {
                flModule.getPosition(),
                frModule.getPosition(),
                blModule.getPosition(),
                brModule.getPosition()
            }
        );

    }


    public Pose2d getOdomPose()
    {
        odometry.update(
            gyroIO.getHeading(), 
            new SwerveModulePosition[] 
            {
                flModule.getPosition(),
                frModule.getPosition(),
                blModule.getPosition(),
                brModule.getPosition()
            }
        );

        return odometry.getPoseMeters();
    }

    public void updateModuleStates()
    {
        //moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        for (int module = 0; module <= 3; module++)
        {
            modules[module].runToState
            (
                kinematics.toSwerveModuleStates(chassisSpeeds)[module]
            );
        }
    }


    public void updateModuleSpeeds()
    {
        for (int module = 0; module <= 3; module++)
        {
            modules[module].update();
        }
    }

    //fl - 1
    //fr - 2
    //bl - 3
    //br - 4

    @Override
    public void periodic()
    {
        RobotState.getInstance().setPose(getOdomPose());

        updateModuleStates();

        updateModuleSpeeds();
    }

    public Command driveFieldCentric(
        Drive drive,
        DoubleSupplier x,
        DoubleSupplier y,
        DoubleSupplier rotation
    )
    {
        return Commands.run(
         () ->
            {
                Rotation2d side =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red
                      ? new Rotation2d(0)
                      : new Rotation2d(Math.PI);

                drive.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds
                (
                    new ChassisSpeeds(
                        x.getAsDouble(),
                        y.getAsDouble(),
                        rotation.getAsDouble()
                    ),
                    RobotState.getInstance().getRotation().plus(side)
                );                
            }
         , drive);
    }

    public SwerveModuleState[] getModuleStates()
    {
        return new SwerveModuleState[] {
            flModule.getModuleState(),
            frModule.getModuleState(),
            blModule.getModuleState(),
            brModule.getModuleState()
        };
    }


    public static Command driveRobotCentric
    (
        Drive drive,
        DoubleSupplier x,
        DoubleSupplier y,
        DoubleSupplier rotation
    )

    {
        return Commands.run
        (
            () ->
            {
                drive.chassisSpeeds = new ChassisSpeeds(
                    x.getAsDouble(),
                    y.getAsDouble(),
                    rotation.getAsDouble()
                );                
            },
            drive
        );
    }

}

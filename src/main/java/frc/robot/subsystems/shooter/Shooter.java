package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.Hood.Hood;
import frc.robot.subsystems.shooter.Hood.HoodIO;
import frc.robot.subsystems.shooter.Turret.TurretIO;
import frc.robot.subsystems.shooter.Turret.TurretIO.TurretIOInputs;
import frc.robot.subsystems.vision.Vision;

public class Shooter extends SubsystemBase {
    
    @AutoLogOutput(key = "poo/turretPose")
    public Pose2d turretPose;

    public PIDController turretPID;

    public Vision camera;

    public TurretIO turretIO;

    @AutoLogOutput(key = "poo/desiredRotation")
    public Rotation2d desiredRotation = new Rotation2d();

    public TurretIOInputs inputs = new TurretIOInputs();

    Timer timer;

    public Hood hood;
    
    public Shooter(
        TurretIO turretIO,
        HoodIO hoodIO,

        double turretP,
        double turretI,
        double turretD,

        double hoodP,
        double hoodI,
        double hoodD,

        Vision vision
    )
    {
        //hood = new Hood(hoodIO, hoodP, hoodI, hoodD);
        timer = new Timer();
        timer.start();

        this.turretIO = turretIO;
        turretPID = new PIDController(turretP, turretI, turretD, 0.02d);

        hoodIO.setPower(1);

        if (vision != null) camera = vision;

    }


    public void setDesiredRotation(Rotation2d rotation)
    {
        // defines desired rotation.
        // minus 
        double adjustedAngle = 
        (
            (rotation
            .minus(RobotState.getInstance().getRotation())
            .getDegrees()
            % Constants.turretLimit)
        );

        //relates definition to desired rotation.
        desiredRotation = new Rotation2d(Math.toRadians(adjustedAngle));

        //sets PID goal to desired rotation
        turretPID.setSetpoint(adjustedAngle);
    }

    // finds x and y from robot to hub
    // then calculates shortest distance
    // and angle
    public Rotation2d odometryAutoalign()
    {

        Translation2d hubDistance = 
            RobotState.getInstance().getPose().getTranslation()
            .minus(Constants.FieldConstants.getHub().getTranslation());

        double y = RobotState.getInstance().getPose().getY() - Constants.FieldConstants.getHub().getY();//- 4.034;
        double x = RobotState.getInstance().getPose().getX() - Constants.FieldConstants.getHub().getX();
        // hypotenuse = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2)) * Math.signum(y);


       // Rotation2d angle = new Rotation2d(Math.asin(
       //   x / hypotenuse
       // ));

        Rotation2d angle = new Rotation2d(Math.atan(
            y / x
        ));

        if (x > 0) angle = angle.plus(new Rotation2d(Math.PI));
        


        return angle;
    }

    public Rotation2d visionAutoalign()
    {
        double x = LimelightHelpers.getTX(Constants.camName);
        return new Rotation2d(turretIO.getRotation().getDegrees() + x);
    }

    public void autoalign()
    {
        if (LimelightHelpers.getTV(Constants.camName))
        {
            setDesiredRotation(visionAutoalign());
        }
        else
        {
            setDesiredRotation(odometryAutoalign());
        }
    }

    public void update(Timer timer)
    {
        turretIO.updateInputs(inputs, timer);


        turretPID.setPID
        (
            Constants.SimConstants.turretP.get(), 
            Constants.SimConstants.turretI.get(), 
            Constants.SimConstants.turretD.get()
        );

        
        double output = turretPID.calculate(turretIO.getRotation().getDegrees());

        if (Double.isNaN(output))
        {
            output = 0;
        }


        turretIO.setPower
        (
            output
        );


        turretPose = new Pose2d(
            RobotState.getInstance().getPosition(),
            inputs.currentRotation.plus(RobotState.getInstance().getRotation())
        );

    }
    

    @Override
    public void periodic()
    {
        update(timer);
    }

    public static Command joystickCommand(Shooter shooter, DoubleSupplier rotation)
    {
        return Commands.run(
            () ->
            {
                shooter.turretIO.setPower(rotation.getAsDouble());
            }
    
            , shooter);
    }

    public static Command autoAlignCommand(Shooter shooter)
    {
        return Commands.run(
            () ->
            {
                shooter.autoalign();
            }
    
            , shooter);
    }


}

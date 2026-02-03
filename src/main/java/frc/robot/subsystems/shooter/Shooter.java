package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Hood.Hood;
import frc.robot.subsystems.shooter.Hood.HoodIO;
import frc.robot.subsystems.shooter.Turret.Turret;
import frc.robot.subsystems.shooter.Turret.TurretIO;

public class Shooter extends SubsystemBase {
    
    Timer timer;

    public Hood hood;

    public Turret turret;
    
    public Shooter(
        TurretIO turretIO,
        HoodIO hoodIO,

        double turretP,
        double turretI,
        double turretD,

        double hoodP,
        double hoodI,
        double hoodD
    )
    {
        hood = new Hood(hoodIO, hoodP, hoodI, hoodD);

        turret = new Turret(turretIO, turretP, turretI, turretD, null);

        timer = new Timer();
        timer.start();
    }

    

    @Override
    public void periodic()
    {
        turret.update(timer);
    }

    public static Command joystickCommand(Shooter shooter, DoubleSupplier rotation)
    {
        return Commands.run(
            () ->
            {
                shooter.turret.turretIO.setPower(rotation.getAsDouble());
            }
    
            , shooter);
    }

    public static Command rotateToSetpoint(Shooter shooter)
    {
        return Commands.runOnce(
            () ->
            {
                shooter.turret.setDesiredRotation(
                    shooter.turret.getRotation().plus(new Rotation2d(Math.PI / 2))
                );
            }
        , shooter);
    }

    public static Command autoAlignCommand(Shooter shooter)
    {
        return Commands.run(
            () ->
            {
                shooter.turret.autoalign();
            }
    
            , shooter);
    }


}

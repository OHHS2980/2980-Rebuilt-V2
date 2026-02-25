package frc.robot.subsystems.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
   
    public final IntakeIO motors;
    public final PIDController intakePID;
    final double kP = 0;
    final double kI = 0;
    final double kD = 0;
    double desiredAngle;

    public Intake (IntakeIO motors ) { 
        this.motors = motors;
        intakePID = new PIDController(kP, kI, kD);
    }

    public void setAngle (double angle) {
        intakePID.setSetpoint(angle);
    }

    public void update() {
        desiredAngle = intakePID.calculate(motors.getRotation().getDegrees());
    }

    public Command intake2Foward() {
        return Commands
        .startRun(() -> setAngle(45), () -> motors.intake2SetPower(desiredAngle), this)
        .until(() -> intakePID.getError() <= 1)
        .finallyDo(() -> motors.intake2SetPower(0));

    }

    public Command intake1Start() {
        return Commands.run(() -> motors.intake1SetPower(1), this);
    }

    public Command intake1Stop() {
        return Commands.runOnce(() -> motors.intake1SetPower(0), this);
    }
}
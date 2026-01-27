package frc.robot.subsystems.shooter.Hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.Turret.TurretIO.TurretIOInputs;

public class HoodIOSim implements HoodIO {
    
    public DCMotorSim motor;

    public Timer timer;

    public double lastTime = 0;

    public HoodIOSim()
    {
        timer = new Timer();
        timer.start();
        
        motor = new DCMotorSim
        (
            LinearSystemId.createDCMotorSystem(
                0.25,0.25
            ),
            DCMotor.getNeo550(1)
        );

        //motor.setInputVoltage(1);
            
    }

    @Override
    public Rotation2d getRotation()
    {
        return new Rotation2d(motor.getAngularPosition());
    }

    @Override
    public void setPower(double power) {
        motor.setInputVoltage(power * 12);
    }
    
    public void updateInputs(TurretIOInputs inputs)
    {
        motor.update(timer.get() - lastTime);
        
        lastTime = timer.get();

        

        inputs.currentRotation = getRotation();
    }
}

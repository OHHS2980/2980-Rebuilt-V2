package frc.robot.subsystems.shooter.Turret;

import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class TurretIOSim implements TurretIO {

    public DCMotorSim motor;

    public double lastTime = 0;

    public TurretIOSim()
    {
        motor = new DCMotorSim
        (
            LinearSystemId.createDCMotorSystem(
                .25,.25
            ),
            DCMotor.getNeo550(2), 
            new double[] {0,0}
        );
            
    }


    @Override
    public Rotation2d getRotation()
    {
        return new Rotation2d(motor.getAngularPositionRad());
    }

    @Override
    public void setPower(double power) {
        motor.setInputVoltage(power * 12);
    }
    
    public void updateInputs(TurretIOInputs inputs, Timer timer)
    {
        motor.update(timer.get() - lastTime);
        lastTime = timer.get();

        
        inputs.currentRotation = getRotation();
    }
}

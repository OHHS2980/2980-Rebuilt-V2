package frc.robot.subsystems.Turret;

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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {

    public DCMotorSim motor;


    public TurretIOSim()
    {
        motor = new DCMotorSim
        (
            LinearSystemId.createDCMotorSystem(
                1,1
            ), 
            DCMotor.getNeo550(1), 
            new double[] {0,0}
        );
            
    }


    @Override
    public Rotation2d getRotation()
    {
        return new Rotation2d(motor.getAngularPosition());
    }

    @Override
    public void setPower(double power) {
        motor.setInputVoltage(power);
    }
    
    public void updateInputs(TurretIOInputs inputs)
    {
        inputs.currentRotation = getRotation();
    }
}

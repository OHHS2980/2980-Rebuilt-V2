package frc.robot.subsystems.indexer.agitator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AgitatorIONeo implements AgitatorIO {
  SparkMax agitatorMotor1;
  SparkMax agitatorMotor2;
  SparkMaxConfig agitatorConfig;
  RelativeEncoder agitator1Encoder;
  RelativeEncoder agitator2Encoder;

  public AgitatorIONeo() {
    agitatorMotor1 = new SparkMax(999999, MotorType.kBrushless);
    agitatorMotor1.configure(
        new SparkMaxConfig(),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    agitatorMotor2 = new SparkMax(999999, MotorType.kBrushless);
    agitatorMotor2.configure(
        new SparkMaxConfig(),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    agitator1Encoder = agitatorMotor1.getEncoder();
    agitator2Encoder = agitatorMotor2.getEncoder();
  }

  @Override
  public void updateInputs(AgitatorIOInputs inputs) {

    inputs.agitator1Voltage = agitatorMotor1.getBusVoltage();
    inputs.agitator1Current = agitatorMotor1.getOutputCurrent();
    inputs.agitator1Position = agitator1Encoder.getPosition();
    inputs.agitator1Velocity = agitator1Encoder.getVelocity();
    inputs.agitator1TempC = agitatorMotor1.getMotorTemperature();

    inputs.agitator2Voltage = agitatorMotor2.getBusVoltage();
    inputs.agitator2Current = agitatorMotor2.getOutputCurrent();
    inputs.agitator2Position = agitator2Encoder.getPosition();
    inputs.agitator2Velocity = agitator2Encoder.getVelocity();
    inputs.agitator2TempC = agitatorMotor2.getMotorTemperature();

  }

  @Override
  public void set1Voltage(double voltage) {
    agitatorMotor1.setVoltage(voltage);
  }

    public void set2Voltage(double voltage) {
    agitatorMotor2.setVoltage(voltage);
  }
}

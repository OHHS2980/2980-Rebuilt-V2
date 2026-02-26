package frc.robot.subsystems.indexer.agitator;

import org.littletonrobotics.junction.AutoLog;

public interface AgitatorIO {
  @AutoLog
  class AgitatorIOInputs {
    public boolean agitator1Connected = false;
    public boolean agitator2Connected = false;

    public double agitator1Voltage = 0.0;
    public double agitator1Current = 0.0;
    public double agitator1Position = 0.0;
    public double agitator1Velocity = 0.0;
    public double agitator1TempC = 0.0;
    
    public double agitator2Voltage = 0.0;
    public double agitator2Current = 0.0;
    public double agitator2Position = 0.0;
    public double agitator2Velocity = 0.0;
    public double agitator2TempC = 0.0;
  }
  

  public default void updateInputs(AgitatorIOInputs inputs) {

  }

  public default void set1Voltage(double voltage) {
    }

  public default void set2Voltage(double voltage) {
    }
}

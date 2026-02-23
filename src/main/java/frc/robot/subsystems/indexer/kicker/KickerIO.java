package frc.robot.subsystems.indexer.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  class KickerIOInputs {
    public boolean kickerConnected = false;

    public double kickerVoltage = 0.0;
    public double kickerCurrent = 0.0;
    public double kickerPosition = 0.0;
    public double kickerVelocity = 0.0;
    public double kickerTempC = 0.0;
  }
  

  public default void updateInputs(KickerIOInputs inputs) {

  }

  public default void setVoltage(double voltage) {

  }
}

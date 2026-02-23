package frc.robot.subsystems.indexer.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.kicker.KickerIO.KickerIOInputs;

import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputs inputs = new KickerIOInputs();

  public Kicker(KickerIO io) {
    this.io = io;
  }


  @Override
  public void periodic()
  {
      io.updateInputs(inputs);
  }

  public Command stop()
  {
      return Commands.run(() -> { 
        io.setVoltage(0); 
        }, 
        this);
  }

  public Command kick()
  {
      return Commands.run(() -> { 
        io.setVoltage(1); 
        }, 
        this);
  }

}

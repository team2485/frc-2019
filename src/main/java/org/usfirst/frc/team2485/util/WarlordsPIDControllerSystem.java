package org.usfirst.frc.team2485.util;

public class WarlordsPIDControllerSystem extends WarlordsControlSystem {
  private WarlordsPIDController centralPIDController;
  private WarlordsPIDController maxOutputRangePIDController;
  private WarlordsPIDController minOutputRangePIDController;
  
  public WarlordsPIDControllerSystem(WarlordsPIDController centralPIDController, WarlordsPIDController maxOutputRangePIDController, WarlordsPIDController minOutputRangePIDController) { this.centralPIDController = centralPIDController;
    this.maxOutputRangePIDController = maxOutputRangePIDController;
    this.minOutputRangePIDController = minOutputRangePIDController;
  }
  
  protected void calculate()
  {
    maxOutputRangePIDController.calculate();
    minOutputRangePIDController.calculate();
    centralPIDController.calculate();
    
    if (centralPIDController.getOutput() >= maxOutputRangePIDController.getOutput()) {
      centralPIDController.setIntegralTerm(maxOutputRangePIDController.getIntegralTerm());
      minOutputRangePIDController.setIntegralTerm(maxOutputRangePIDController.getIntegralTerm());
    } else if (centralPIDController.getOutput() <= minOutputRangePIDController.getOutput()) {
      centralPIDController.setIntegralTerm(minOutputRangePIDController.getIntegralTerm());
      maxOutputRangePIDController.setIntegralTerm(minOutputRangePIDController.getIntegralTerm());
    } else {
      maxOutputRangePIDController.setIntegralTerm(centralPIDController.getIntegralTerm());
      minOutputRangePIDController.setIntegralTerm(centralPIDController.getIntegralTerm());
    }
  }
}
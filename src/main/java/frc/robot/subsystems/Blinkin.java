// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase {
  /** Creates a new Blinkin. */
  private double color = 0;
  private Spark spark;
  public Blinkin() {
      spark = new Spark(0);
  }


  public void setColor(double num){
      color = num;
  }
  public void setRed(){
    color = 0.81;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    spark.set(color);
  }
}

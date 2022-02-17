// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Counter;
import static frc.robot.Constants.LIDAR_DATA.*;
import static frc.robot.Constants.GAME_DATA.*;
import frc.robot.MathFunction;

public class Lidar extends SubsystemBase {
  private Counter counter;
  private static final double offset = 0.0;
  private static final double factor = 1;

  /** Creates a new Lidar. */
  public Lidar() {
    counter = new Counter(0);
    counter.setMaxPeriod(1);
    counter.setSemiPeriodMode(true);
    counter.reset();
  }

  public double getDistance() {
    if (counter.get() < 1) {
      return 0.0;
    }
    else{
      return (counter.getPeriod()*factor + offset);
    }
  }

  public double getAbsoluteDistance(double angle) {
    return this.getDistance()*Math.cos(angle);
  }

  public double getAbsoluteHeight(double angle) {
    return this.getDistance()*Math.sin(angle);
  }

  public double getVxVelocity(double angle_shooter) {
    return this.getAbsoluteDistance(angle_shooter)*driff_coefficent*ball_diameter/ball_mass;
  }

  public double getVyVelocity(double angle_shooter, double height) {
    double b = height*Math.pow(driff_coefficent/ball_mass, 2) - gravitational_accel;
    return ((MathFunction.solving_equation(gravitational_accel, 3, b) - gravitational_accel)/(driff_coefficent/ball_mass));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;

//Import required WPILib libraries
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;

public class Intake extends SubsystemBase {
  /** Creates a new Shooter. */
  private SparkMax motor;
  private RelativeEncoder encoder;
  private SparkClosedLoopController pid;
  private double desiredVelocity = 0;
  //Create an instance of the AnalogInput class so we can read from it later
  public AnalogInput ultrasonicSensor = new AnalogInput(0); // TODO: Check channel
  public DigitalOutput ultrasonicSensorTrigger = new DigitalOutput(0);
  public double ultrasonicSensorRange = 0;
  public double voltageScaleFactor = 1;

  public Intake() {

    motor = new SparkMax(kCANID, SparkMax.MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    config.encoder
        .velocityConversionFactor(1);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(0, 0, 0, 0.000175);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake", encoder.getVelocity());
    SmartDashboard.putNumber("Desired Intake", desiredVelocity);
    SmartDashboard.putNumber("Ultra Sensor", ultrasonicSensorRange);

    pid.setReference(desiredVelocity, ControlType.kVelocity);

    voltageScaleFactor = 5/RobotController.getVoltage5V(); //Calculate what percentage of 5 Volts we are actually at

    // Check sensor
    
    if (desiredVelocity != 0)
    {
      ultrasonicSensorRange = ultrasonicSensor.getValue()*voltageScaleFactor*0.125;

      if (ultrasonicSensorRange < distanceStop)
      {
        desiredVelocity = 0;
        ultrasonicSensorTrigger.set(false);
      }
    }
  }
 
  public Command Intake_coral() {
    return runOnce(() -> {
      desiredVelocity = 100;
      ultrasonicSensorTrigger.set(true);
    });
  }

  public Command Outtake() {
    return runEnd(() -> {
      desiredVelocity = -5820;
    }, () -> {
      desiredVelocity = 0;
    });
  }

  public Command autoShoot(double time) {
    return runEnd(() -> {
      desiredVelocity = 5820 * 0.75;
    }, () -> {
      desiredVelocity = 0;
    }).withTimeout(time);
  }
}

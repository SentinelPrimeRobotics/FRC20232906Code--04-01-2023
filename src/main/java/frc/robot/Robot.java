// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private XboxController xbox = new XboxController(0);

  
  private final MotorController switch1 = new Talon(2);
  private final MotorController switch2 = new Talon(3);
  private final MotorController m_rightMotor = new Talon(0);
  private final MotorController m_rightMotor2 = new Talon(1); 
  private final WPI_VictorSPX m_leftMotor = new WPI_VictorSPX(6);
  private final WPI_VictorSPX m_leftMotor2 = new WPI_VictorSPX(7);

  private final MotorControllerGroup left = new MotorControllerGroup(m_leftMotor, m_leftMotor2);
  private final MotorControllerGroup right = new MotorControllerGroup( m_rightMotor, m_rightMotor2);
  private final MotorControllerGroup armMotors = new MotorControllerGroup(switch1, switch2);
 
  private double startTime;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltj   ages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    right.setInverted(true);

    m_myRobot = new DifferentialDrive(left, right);

  }

@Override
public void autonomousInit() {
  startTime = Timer.getFPGATimestamp();
}

@Override
public void autonomousPeriodic() {
  double time = Timer.getFPGATimestamp();
  System.out.println(time - startTime);

  if (time-startTime < 3) {
    m_rightMotor.set(0.4);
    m_rightMotor2.set(0.4);
    m_leftMotor.set(-0.4);
    m_leftMotor2.set(-0.4);
  }
  else {
    m_rightMotor.set(0);
    m_rightMotor2.set(0);
    m_leftMotor.set(0);
    m_leftMotor2.set(0);
  }
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.arcadeDrive(xbox.getLeftY(), xbox.getRightX());
    if (xbox.getAButton()) {
      armMotors.set(0.35);

    }else if (xbox.getBButton()) {
      armMotors.set(-0.2);
    } else {
      armMotors.set(0);

    }  
    


    
  }
}



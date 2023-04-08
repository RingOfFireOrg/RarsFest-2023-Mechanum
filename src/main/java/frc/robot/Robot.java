// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private double speed;
  private double strafe;
  private double turn;
  private double leftFront;
  private double rightFront;
  private double leftRear;
  private double rightRear;
  private double startTime;
  

  private final MotorController frontLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final MotorController frontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final MotorController backLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final MotorController backRightMotor = new CANSparkMax(4, MotorType.kBrushless);
  //private MotorControllerGroup blue = new MotorControllerGroup(m_frontleftMotor, m_backleftMotor );
  //private MotorControllerGroup green = new MotorControllerGroup(m_frontrightMotor, m_backrightMotor );
 

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightMotor.setInverted(true);

    //m_myRobot = new DifferentialDrive(blue, green);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

  }

   {
  }


public void moveForward() {
      frontLeftMotor.set(0.2);
      frontRightMotor.set(0.2);
      backLeftMotor.set(0.2);
      backRightMotor.set(0.2);
}

public void moveRight() {
    frontLeftMotor.set(-0.2);
    frontRightMotor.set(0.2);
    backLeftMotor.set(0.2);
    backRightMotor.set(-0.2);
}

public void moveStop() {
    frontLeftMotor.set(0);
    frontRightMotor.set(0);
    backLeftMotor.set(0);
    backRightMotor.set(0);
}

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp();
    if (time-startTime < 1) {
        moveForward();
    }else if (time-startTime > 1 && time-startTime < 2) {
        moveRight();
    } else {
        moveStop();
    }

  }

  @Override
  public void teleopPeriodic() {
   //m_myRobot.tankDrive(m_leftStick.getY()*.5, m_rightStick.getY()*.5);

    speed = -m_leftStick.getY();
    strafe = m_leftStick.getX();
    turn = m_leftStick.getTwist();

    leftFront = speed + turn + strafe;
    rightFront = speed - turn - strafe;
    leftRear = speed + turn - strafe;
    rightRear = speed - turn + strafe;
    
    frontLeftMotor.set(leftFront*0.5);
    frontRightMotor.set(rightFront*0.5);
    backLeftMotor.set(leftRear*0.5);
    backRightMotor.set(rightRear*0.5);

  }
}

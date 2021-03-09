/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static Drive drive;
  private static XboxController xc = new XboxController(0);

  int smoothing = 0;
  int pov = -1;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    drive = new Drive(0, 1, 2, 3, 4, 5);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    double targetPos = 4096 * 3;
    drive.run(ControlMode.MotionMagic, targetPos);
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
  }


  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double rightY = -xc.getY(Hand.kRight); /* left-side Y for Xbox360Gamepad */
    double leftY = -xc.getY(Hand.kLeft); /* right-side Y for Xbox360Gamepad */
    if (Math.abs(leftY) < 0.10) { leftY = 0; } /* deadband 10% */
    if (Math.abs(rightY) < 0.10) { rightY = 0; } /* deadband 10% */

    if (xc.getRawButton(5)) {
      double targetPos = leftY * 4096;
      drive.run(ControlMode.MotionMagic, targetPos);
      System.out.println(targetPos);
    } 
    else {
      drive.run(ControlMode.PercentOutput, leftY);
    }
    if (xc.getRawButton(6)) {
      drive.setPos(0);
    }

    int savePov = xc.getPOV();
    if (pov == savePov) {
    } 
    else if (pov == 180) {
      smoothing--;
      if (smoothing < 0)
        smoothing = 0;
      drive.configMotionSCurveStrength(smoothing);
    } 
    else if (pov == 0) { 
      smoothing++;
      if (smoothing > 8)
        smoothing = 8;
      drive.configMotionSCurveStrength(smoothing);
    }
    pov = savePov; 
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

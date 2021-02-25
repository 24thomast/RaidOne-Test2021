package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;

public class Drive{
    private static TalonSRX talon0;
    private static TalonSRX talon1;
    private static TalonSRX talon2;
    private static TalonSRX talon3;
    private static TalonSRX talon4;
    private static TalonSRX talon5;

    public Drive(int tal0, int tal1, int tal2, int tal3, int tal4, int tal5){
        talon0 = new TalonSRX(tal0);
        talon1 = new TalonSRX(tal1);
        talon2 = new TalonSRX(tal2);
        talon3 = new TalonSRX(tal3);
        talon4 = new TalonSRX(tal4);
        talon5 = new TalonSRX(tal5);

        talon0.configFactoryDefault();
        talon1.configFactoryDefault();
        talon2.configFactoryDefault();
        talon3.configFactoryDefault();
        talon4.configFactoryDefault();
        talon5.configFactoryDefault();

        talon0.setNeutralMode(NeutralMode.Brake);
        talon1.setNeutralMode(NeutralMode.Brake);
        talon2.setNeutralMode(NeutralMode.Brake);
        talon3.setNeutralMode(NeutralMode.Brake);
        talon4.setNeutralMode(NeutralMode.Brake);
        talon5.setNeutralMode(NeutralMode.Brake);

        talon3.follow(talon1);
        talon5.follow(talon1);
        talon2.follow(talon0);
        talon4.follow(talon0);

        talon1.setInverted(true);
        talon3.setInverted(true);
        talon5.setInverted(true);

        /* Set relevant frame periods to be at least as fast as periodic rate */
		talon0.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		talon0.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

        talon1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		talon1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		talon0.configNominalOutputForward(0, Constants.kTimeoutMs);
		talon0.configNominalOutputReverse(0, Constants.kTimeoutMs);
		talon0.configPeakOutputForward(1, Constants.kTimeoutMs);
		talon0.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        talon1.configNominalOutputForward(0, Constants.kTimeoutMs);
		talon1.configNominalOutputReverse(0, Constants.kTimeoutMs);
		talon1.configPeakOutputForward(1, Constants.kTimeoutMs);
		talon1.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		talon0.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		talon0.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		talon0.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		talon0.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		talon0.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

        talon1.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		talon1.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		talon1.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		talon1.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		talon1.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);


		/* Set acceleration and vcruise velocity - see documentation */
		talon0.configMotionCruiseVelocity(3000, Constants.kTimeoutMs);
		talon0.configMotionAcceleration(3000, Constants.kTimeoutMs);

        talon1.configMotionCruiseVelocity(3000, Constants.kTimeoutMs);
		talon1.configMotionAcceleration(3000, Constants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		talon0.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        talon1.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }

    public double motorOutput0(){
        return talon0.getMotorOutputPercent();
    }

    public double motorOutput1(){
        return talon1.getMotorOutputPercent();
    }

    public void setSelectedSensorPos(double val){
        talon0.setSelectedSensorPosition(val);
        talon1.setSelectedSensorPosition(val);
    }

    public void configMotionSCurveStrength(int smoothing){
        talon0.configMotionSCurveStrength(smoothing);
        talon1.configMotionSCurveStrength(smoothing);
    }
}


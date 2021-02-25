package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import frc.robot.sim.PhysicsSim;

public class DriveTest {
	/* Hardware */
	TalonSRX _talon = new WPI_TalonSRX(1);
	Joystick _joy = new Joystick(0);

	/* create some followers */
	BaseMotorController _follower1 = new TalonSRX(0);
	BaseMotorController _follower2 = new VictorSPX(0);
	BaseMotorController _follower3 = new VictorSPX(1);

	/* Used to build string throughout loop */
	StringBuilder _sb = new StringBuilder();

	/** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing = 0;

	/** save the last Point Of View / D-pad value */
	int _pov = -1;

	public void simulationInit() {
		PhysicsSim.getInstance().addTalonSRX(_talon, 0.75, 5100, false);
	}
	public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
	}

	public void robotInit() {
		/* setup some followers */
		_follower1.configFactoryDefault();
		_follower2.configFactoryDefault();
		_follower3.configFactoryDefault();
		_follower1.follow(_talon);
		_follower2.follow(_talon);
		_follower3.follow(_talon);

		/* Factory default hardware to prevent unexpected behavior */
		_talon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		_talon.configNeutralDeadband(0.001, Constants.kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_talon.setSensorPhase(false);
		_talon.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		_talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		_talon.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(3000, Constants.kTimeoutMs);
		_talon.configMotionAcceleration(3000, Constants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		/* Get gamepad axis - forward stick is positive */
		double leftYstick = -1.0 * _joy.getY(); /* left-side Y for Xbox360Gamepad */
		double rghtYstick = -1.0 * _joy.getRawAxis(5); /* right-side Y for Xbox360Gamepad */
		if (Math.abs(leftYstick) < 0.10) { leftYstick = 0; } /* deadband 10% */
		if (Math.abs(rghtYstick) < 0.10) { rghtYstick = 0; } /* deadband 10% */

		/* Get current Talon SRX motor output */
		double motorOutput = _talon.getMotorOutputPercent();

		/* Prepare line to print */
		_sb.append("\tOut%:");
		_sb.append(motorOutput);
		_sb.append("\tVel:");
		_sb.append(_talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));

		/**
		 * Peform Motion Magic when Button 1 is held, else run Percent Output, which can
		 * be used to confirm hardware setup.
		 */
		if (_joy.getRawButton(1)) {
			/* Motion Magic */

			/* 4096 ticks/rev * 10 Rotations in either direction */
			double targetPos = rghtYstick * 4096 * 10.0;
			_talon.set(ControlMode.MotionMagic, targetPos);

			/* Append more signals to print when in speed mode */
			_sb.append("\terr:");
			_sb.append(_talon.getClosedLoopError(Constants.kPIDLoopIdx));
			_sb.append("\ttrg:");
			_sb.append(targetPos);
		} else {
			/* Percent Output */

			_talon.set(ControlMode.PercentOutput, leftYstick);
		}
		if (_joy.getRawButton(2)) {
			/* Zero sensor positions */
			_talon.setSelectedSensorPosition(0);
		}

		int pov = _joy.getPOV();
		if (_pov == pov) {
			/* no change */
		} else if (_pov == 180) { // D-Pad down
			/* Decrease smoothing */
			_smoothing--;
			if (_smoothing < 0)
				_smoothing = 0;
			_talon.configMotionSCurveStrength(_smoothing);

			System.out.println("Smoothing is set to: " + _smoothing);
		} else if (_pov == 0) { // D-Pad up
			/* Increase smoothing */
			_smoothing++;
			if (_smoothing > 8)
				_smoothing = 8;
			_talon.configMotionSCurveStrength(_smoothing);

			System.out.println("Smoothing is set to: " + _smoothing);
		}
		_pov = pov; /* save the pov value for next time */

		/* Instrumentation */
		Instrum.Process(_talon, _sb);
	}
}
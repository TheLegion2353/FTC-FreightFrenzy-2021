package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends RobotPart {
	private double kP = 1.0;
	private double kI = 0.0;
	private double kD = 0.0;
	private double position = 0.0;
	private Telemetry telemetry = null;

	private HardwareControllerEx motorController;
	private PID pid;

	public Arm(Gamepad gp, DcMotorEx motor, Telemetry tel) {
		super(gp);
		telemetry = tel;
		pid = new PID(kP, kI, kD, position);
		motorController = new HardwareControllerEx(tel, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, null, motor);
	}

	@Override
	public void update() {
		if (gamepad != null) {
			position += (gamepad.right_trigger - gamepad.left_trigger) * 100.0 * pid.getElapsedTime();
			if (position > 3.0) {
				position = 3.0;
			}
			if (position < 0.3) {
				position = 0.3;
			}
			pid.setSetPoint(position);
			double voltage = pid.PIDLoop(motorController.getVoltage());
			motorController.setSpeed(voltage);
			telemetry.addData("Analog Input Voltage: ", motorController.getVoltage());
		}
	}

	public void setPotentiometer(AnalogInput pot) {
		motorController.addAnalogInput(pot);
	}
}

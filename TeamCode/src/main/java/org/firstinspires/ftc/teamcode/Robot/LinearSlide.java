package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide extends RobotPart {
	private double kP = 0.0;
	private double kI = 0.0;
	private double kD = 0.0;
	private double position = 0.0;

	private HardwareControllerEx motorController;

	public LinearSlide(Gamepad gp, DcMotorEx motor, Telemetry tel) {
		super(gp);
		motorController = new HardwareControllerEx(tel, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, new PID(kP, kI, kD, position), motor);
	}

	@Override
	public void update() {
		if (gamepad != null) {
			if (gamepad.right_bumper) {
				position += 10.0 * motorController.getPID(0).getElapsedTime();
			} else if (gamepad.left_bumper) {
				position -= 10.0 * motorController.getPID(0).getElapsedTime();
			}
			motorController.setSpeed(position);
		}
	}
}

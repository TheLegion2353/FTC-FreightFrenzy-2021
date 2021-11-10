package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Carousel extends RobotPart {
	private double kP = 0.01;
	private double kI = 0.0;
	private double kD = 0.0;
	private HardwareControllerEx motorController;

	public Carousel(Gamepad gp, DcMotorEx motor, Telemetry tel) {
		super(gp);
		motorController = new HardwareControllerEx(tel, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, new MotorPID(kP, kI, kD, 0.0, tel), motor);
	}

	@Override
	public void update() {
		if (gamepad != null) {
			if (gamepad.dpad_up) {
				motorController.setSpeed(3100);
			} else if (gamepad.dpad_down) {
				motorController.setSpeed(-3100.0);
			} else {
				motorController.setSpeed(0.0);
			}
		}
	}
}

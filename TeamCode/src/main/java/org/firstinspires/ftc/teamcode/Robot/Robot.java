package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Robot {
	private ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
	private Drivetrain mecanum;
	private Carousel carousel;
	private Arm arm;
	private LinearSlide slide;
	private Gamepad gamepad = null;
	private Telemetry telemetry = null;
	private boolean isWaiting = false;
	private double endTime = 0;
	private double xPosition = 0;  // inches
	private double yPosition = 0;
	private double angle = 0;

	public Robot(Gamepad gp, Telemetry t) {
		telemetry = t;
		gamepad = gp;
		mecanum = new Drivetrain(Drivetrain.ControlType.ARCADE, gamepad, t);
	}

	public Robot(Gamepad gp) {
		gamepad = gp;
	}

	public void update() {
		double time = (double)clock.time(TimeUnit.MILLISECONDS) / 1000.0;
		if (mecanum != null) {
			mecanum.update();
		} else {
			telemetry.addLine("Drivetrain null!");
		}

		if (carousel != null) {
			carousel.update();
		} else {
			telemetry.addLine("Carousel null!");
		}

		if (arm != null) {
			arm.update();
		} else {
			telemetry.addLine("Arm null!");
		}

		if (slide != null) {
			slide.update();
		} else {
			telemetry.addLine("Linear slide null!");
		}
		clock.reset();
	}

	public void setDrivetrain(Drivetrain dt) {
		mecanum = dt;
	}

	public void setTopLeft(DcMotorEx ... motors) {
		mecanum.setTopLeft(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, motors);
	}

	public void setTopRight(DcMotorEx ... motors) {
		mecanum.setTopRight(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, motors);
	}

	public void setBottomLeft(DcMotorEx ... motors) {
		mecanum.setBottomLeft(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, motors);
	}

	public void setBottomRight(DcMotorEx ... motors) {
		mecanum.setBottomRight(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, motors);
	}

	public void setCarouselMotor(DcMotorEx motor) {
		carousel = new Carousel(gamepad, motor, telemetry);
	}

	public void setArm(DcMotorEx motor, AnalogInput pot) {
		arm = new Arm(gamepad, motor, telemetry);
		arm.setPotentiometer(pot);
	}

	public void setLinearSlide(DcMotorEx motor) {
		slide = new LinearSlide(gamepad, motor, telemetry);
	}
}
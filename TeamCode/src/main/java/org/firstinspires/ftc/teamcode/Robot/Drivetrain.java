package org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


public class Drivetrain extends RobotPart {
	private final double freeKP = 0.00125;
	private final double freeKI = 0.01;
	private final double freeKD = 0.000002;
	private final double chainedKP = 0.00125;
	private final double chainedKI = 0.01;
	private final double chainedKD = 0.000002;

	private ElapsedTime clock = null;
	private ControlType control;
	private HardwareControllerEx topRight = null;
	private HardwareControllerEx topLeft = null;
	private HardwareControllerEx bottomLeft = null;
	private HardwareControllerEx bottomRight = null;
	private Telemetry telemetry = null;

	public Drivetrain(ControlType ct, Gamepad gp, Telemetry t) {
		super(gp);
		telemetry = t;
		control = ct;
		topLeft = new HardwareControllerEx();
		topRight = new HardwareControllerEx();
		bottomLeft = new HardwareControllerEx();
		bottomRight = new HardwareControllerEx();
	}

	public Drivetrain(ControlType ct, Gamepad gp) {
		super(gp);
		control = ct;
		topLeft = new HardwareControllerEx();
		topRight = new HardwareControllerEx();
		bottomLeft = new HardwareControllerEx();
		bottomRight = new HardwareControllerEx();
	}

	@Override
	protected void driverUpdate() {
		if (gamepad != null) {
			double leftPower = -gamepad.left_stick_y + gamepad.right_stick_x;
			double rightPower = gamepad.left_stick_y + gamepad.right_stick_x;
			double strafePower = -gamepad.left_stick_x;
			topLeft.setSpeed((leftPower - strafePower) * 4000.0 * .65);
			topRight.setSpeed((rightPower - strafePower) * 4000.0);
			bottomLeft.setSpeed((leftPower + strafePower) * 4000.0 * .65);
			bottomRight.setSpeed((rightPower + strafePower) * 4000.0);
		}
	}

	public void setTopLeft(DcMotorEx.RunMode mode, DcMotorEx ... motors) {
		topLeft = new HardwareControllerEx(telemetry, mode, new MotorPID(chainedKP, chainedKI, chainedKD, 0.0, telemetry), motors);
		topLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
	}

	public void setTopRight(DcMotorEx.RunMode mode, DcMotorEx ... motors) {
		topRight = new HardwareControllerEx(telemetry, mode, new MotorPID(freeKP, freeKI, freeKD, 0.0, telemetry), motors);
		topRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
	}

	public void setBottomLeft(DcMotorEx.RunMode mode, DcMotorEx ... motors) {
		bottomLeft = new HardwareControllerEx(telemetry, mode, new MotorPID(chainedKP, chainedKI, chainedKD, 0.0, telemetry), motors);
		bottomLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
	}

	public void setBottomRight(DcMotorEx.RunMode mode, DcMotorEx ... motors) {
		bottomRight = new HardwareControllerEx(telemetry, mode, new MotorPID(freeKP, freeKI, freeKD, 0.0, telemetry), motors);
		bottomRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
	}

	public enum ControlType {
		TANK,
		ARCADE
	}
}
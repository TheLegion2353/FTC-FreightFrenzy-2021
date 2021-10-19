package org.firstinspires.ftc.teamcode.OpModes;
import org.firstinspires.ftc.teamcode.Robot.HardwareController;
import org.firstinspires.ftc.teamcode.Robot.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Main Mecanum", group="Driver Controlled")
public class MainTeleOp extends OpMode {
	private Drivetrain drive = null;
	@Override
	public void init() {
		drive = new Drivetrain(Drivetrain.ControlType.ARCADE, gamepad1, telemetry);
		drive.setTopRight(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, hardwareMap.get(DcMotorEx.class, "topRightDrive"));
		drive.setTopLeft(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, hardwareMap.get(DcMotorEx.class, "topLeftDrive"));
		drive.setBottomRight(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, hardwareMap.get(DcMotorEx.class, "backRightDrive"));
		drive.setBottomLeft(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, hardwareMap.get(DcMotorEx.class, "backLeftDrive"));
	}

	@Override
	public void loop() {
		drive.update();
	}
}
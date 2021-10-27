package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PID1stDerivative extends PID {

	private double lastPos = 0.0;

	public PID1stDerivative(double P, double I, double D, double sp) {
		super(P, I, D, sp);
	}

	public PID1stDerivative(double P, double I, double D, double sp, Telemetry telemetry) {
		super(P, I, D, sp, telemetry);
	}

	@Override
	protected double PIDLoopInternal(double currentVal, double elapsedTime) {
		double currentVel = (currentVal - lastPos) / elapsedTime;
		lastPos = currentVal;
		telemetry.addData("Current Velocity: ", currentVel);
		telemetry.addData("SetPoint", setPoint);
		telemetry.addData("Current Value: ", currentVal);
		return super.PIDLoopInternal(currentVel, elapsedTime);
	}
}

/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class K9TeleOp extends OpMode {

	/*
	 * Note: the configuration of the servos is such that
	 * as the arm servo approaches 0, the arm position moves up (away from the floor).
	 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
	 */
	// TETRIX VALUES.
	final static double ARM_MIN_RANGE  = 0.20;
	final static double ARM_MAX_RANGE  = 0.90;
	final static double CLAW_MIN_RANGE  = 0.20;
	final static double CLAW_MAX_RANGE  = 0.7;

	// position of the arm servo.
	double armPosition;

	// amount to change the arm servo position.
	double armDelta = 0.1;

	// position of the claw servo
	double clawPosition;

	// amount to change the claw servo position by
	double clawDelta = 0.1;

	DcMotor motorRightF;
	DcMotor motorRightB;
	DcMotor motorLeftF;
	DcMotor motorLeftB;
	DcMotor middlemotor;



	Servo claw;
	Servo arm;

	ColorSensor colorSensor;
	TouchSensor touchSensor;
	OpticalDistanceSensor distanceSensor;
	IrSeekerSensor irSeeker;

	/**
	 * Constructor
	 */
	public K9TeleOp() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */
		
		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *   
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
		try {
			motorRightF = hardwareMap.dcMotor.get("frontright");
			motorRightB = hardwareMap.dcMotor.get("backright");
			motorLeftF = hardwareMap.dcMotor.get("frontleft");
			motorLeftB = hardwareMap.dcMotor.get("backleft");
			arm = hardwareMap.servo.get("arm");
			claw = hardwareMap.servo.get("claw");
			middlemotor = hardwareMap.dcMotor.get("middlemotor");
		}catch(Exception e){
			telemetry.addData("",e.getMessage());
		}



		try {
			//motorLeft = hardwareMap.dcMotor.get("motor_1");
			//motorLeft.setDirection(DcMotor.Direction.REVERSE);
		}catch(Exception e){
			telemetry.addData("",e.getMessage());
		}
		/*try {
			arm = hardwareMap.servo.get("servo_1");
		}catch(Exception e){
			telemetry.addData("",e.getMessage());
		}
		try {
			claw = hardwareMap.servo.get("servo_6");
		}catch(Exception e){
			telemetry.addData("",e.getMessage());
		}
		try {
			colorSensor = hardwareMap.colorSensor.get("color_1");
			colorSensor.enableLed(false);
		}catch(Exception e){
			telemetry.addData("",e.getMessage());
		}
		try {
			touchSensor = hardwareMap.touchSensor.get("touch_1");
		}catch(Exception e){
			telemetry.addData("",e.getMessage());
		}
		try {
			distanceSensor = hardwareMap.opticalDistanceSensor.get("distance_1");
		}catch(Exception e){
			telemetry.addData("",e.getMessage());
		}
		try {
			irSeeker = hardwareMap.irSeekerSensor.get("irSeeker_1");
		}catch(Exception e){
			telemetry.addData("",e.getMessage());
		}



		// assign the starting position of the wrist and claw
		armPosition = 0.2;
		clawPosition = 0.2;*/
	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		/*
		 * Gamepad 1
		 * 
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

		// throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
		// 1 is full down
		// direction: left_stick_x ranges from -1 to 1, where -1 is full left
		// and 1 is full right
		float throttle = -gamepad1.left_stick_y;
		float direction = gamepad1.left_stick_x;
		//float right = throttle - direction;
		//float left = throttle + direction;

		// clip the right/left values so that the values never exceed +/- 1
		//right = Range.clip(right, -1, 1);
		//left = Range.clip(left, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		float right = (float) gamepad1.right_stick_y;
		float left = (float) gamepad1.left_stick_y;

		// write the values to the motors
		//motorRight.setPower(0);
		//motorLeft.setPower(0);

		// update the position of the arm.
		try {
			telemetry.addData("Gamepad1 left x", gamepad1.left_stick_x);
			telemetry.addData("Gamepad1 left y", gamepad1.left_stick_y);
			telemetry.addData("Gamepad1 left x", gamepad1.right_stick_x);
			telemetry.addData("Gamepad1 left y", gamepad1.right_stick_y);
			//telemetry.addData("Gamepad1 )

			motorRightF.setPower(right);
			motorRightB.setPower(-right);

			motorLeftF.setPower(-left);
			motorLeftB.setPower(left);

			if (gamepad2.a) {
				// if the A button is pushed on gamepad1, increment the position of
				// the arm servo.
				armPosition += armDelta;
			}

			if (gamepad2.y) {
				// if the Y button is pushed on gamepad1, decrease the position of
				// the arm servo.
				armPosition -= armDelta;
			}

			// update the position of the claw
			if (gamepad2.x) {
				clawPosition += clawDelta;
			}

			if (gamepad2.b) {
				clawPosition -= clawDelta;
			}

			if(gamepad1.left_bumper){
				middlemotor.setPower(.75);
				telemetry.addData("Gamepad1 left bumper", gamepad1.left_bumper);
			}
			else if(gamepad1.right_bumper){
				middlemotor.setPower(-.75);
				telemetry.addData("Gamepad1 right bumper", gamepad1.right_bumper);
			}
			else{
				middlemotor.setPower(0);
			}

		}catch(Exception e) {
			telemetry.addData("Error", "Cannot detect game controller");
		}

		// clip the position values so that they never exceed their allowed range.
		armPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
		clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

		// write position values to the wrist and claw servo
		arm.setPosition(armPosition);
		claw.setPosition(clawPosition);

		// get sensors and send values back to Driver Station via Telemetry
		/*boolean isTouch = touchSensor.isPressed();
		int blue = colorSensor.blue();
		int green = colorSensor.green();
		int red = colorSensor.red();
		double distance = distanceSensor.getLightDetected();


		// if color sensor detects green, move motors
		// 		motorRight.setPower(right); 		motorLeft.setPower(left);
		if (green > 1) {
			right = (float)0.1;
			motorRight.setPower(right);
		} else {
			motorRight.setPower(0);
		}*/




		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
		telemetry.addData("Text", "Hello Driver");
		telemetry.addData("Text", "*** Robot Data***");
		/*telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
		telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));*/
		telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
		telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
		/*telemetry.addData("Touch Sensor: ", Boolean.toString(isTouch));
		telemetry.addData("Color",String.format("r%d,g%d,b%d",red, green, blue));
		telemetry.addData("Distance: ",distance);*/

	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}


	/*
	 * This method scales the joystick input so for low joystick values, the 
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}

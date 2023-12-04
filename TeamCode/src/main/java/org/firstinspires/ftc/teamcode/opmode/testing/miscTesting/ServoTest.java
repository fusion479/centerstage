/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmode.testing.miscTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Axon Encoder Test", group = "testing")
@Config
public class ServoTest extends LinearOpMode {

    AnalogInput axonBoard;
    Servo axon;
    double reading;
    double x = 0;


    @Override
    public void runOpMode() {
        // MAX = 1;
        // MIN = 0;
        // rampUp = true;
        // Set Analog
        axonBoard =  hardwareMap.get(AnalogInput.class, "axonBoard");
        // axon = hardwareMap.get(Servo.class, "axon");
        // axon.setPosition(0);

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){
            reading = axonBoard.getVoltage() / 3.3 * 360;
            telemetry.addData("Encoder Reading 0.4 ", "%5.2f", reading);
            axon.setPosition(0.2);
            reading = axonBoard.getVoltage() / 3.3 * 360;
            telemetry.addData("Encoder Reading 0.2 ", "%5.2f", reading);
                telemetry.update();
/*
            if (rampUp) {
                while(position <= MAX) {
                    axon.setPosition(position);
                    reading = getReading();
                    telemetry.addData("Servo Position", "%5.2f", position);
                    telemetry.addData("Encoder Reading", "%5.2f", reading);
                    telemetry.update();
                    position += 0.01;
                    sleep(50);
                }
                rampUp = !rampUp;
            }
            else {
                while(position >= MIN) {
                    axon.setPosition(position);
                    reading = getReading();
                    telemetry.addData("Servo Position", "%5.2f", position);
                    telemetry.addData("Encoder Reading", "%5.2f", reading);
                    telemetry.update();
                    position -= 0.01;
                    sleep(50);
                }
                rampUp = !rampUp;
            }
*/

            // Display the current value

        }


        // Signal done;

        telemetry.addData(">", "Done");
        telemetry.update();
    }
}

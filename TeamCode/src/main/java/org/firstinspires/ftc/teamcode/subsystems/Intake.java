package org.firstinspires.ftc.teamcode.subsystems;

import android.widget.ToggleButton;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public boolean toggle = false;
    public static float intakeVoltage = 1;
    public static float idleVoltage = 0;

    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(Motor.class, "intakeMotor");
    }


    public static void intakeToggle() {
        while () // controller logic idk
        {
            toggle = true;
        }
        else toggle = false;
    }

    public void intakeMotorVoltage() {
        if (toggle) {
            intakeMotor.voltage = intakeVoltage;
        } else intakeMotor.voltage = idleVoltage;
    }
}
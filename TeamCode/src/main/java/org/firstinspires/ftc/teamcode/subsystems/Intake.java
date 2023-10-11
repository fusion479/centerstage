package org.firstinspires.ftc.teamcode.subsystems;

import android.widget.ToggleButton;

public class Intake {
    public boolean Toggle = false;
    public static float intakeVoltage = ;//idfk
    public static float idleVoltage = 0;

    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(Motor.class, "intakeMotor");
    }
}

    public static void intakeToggle()
{
        while // controller logic idk
        {
        Toggle = true;
        }
        else Toggle = flase;
        }
public static void intakeMotorVoltage()
        {
            if (Toggle) {
            }
            intakeMotor.voltage = intakeVoltage;
            }
            else

        }

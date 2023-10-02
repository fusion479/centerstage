package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private DcMotorEx[] motors = new DcMotorEx[2];

    public Lift(HardwareMap hwMap) {
        motors[0] = hwMap.get(DcMotorEx.class, "leftLift");
        motors[1] = hwMap.get(DcMotorEx.class, "rightLift");
    }

    public void goBottom() {

    }

    public void goLow() {

    }

    public void goMedium() {

    }

    public void goHigh() {

    }

    public void setHeight() {

    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Subsystem;

@Config
public class Launcher extends Subsystem {
    private final Servo servo;

    public Launcher(final HardwareMap hwMap, final MultipleTelemetry telemetry) {
        super(telemetry);

        this.servo = hwMap.get(Servo.class, "launcher");
    }

    public void launch() {
        this.servo.setPosition(0.6);
    }

    public void idle() {
        this.servo.setPosition(0.15);
    }
}
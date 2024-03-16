package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Subsystem extends SubsystemBase {
    private final MultipleTelemetry telemetry;

    public Subsystem(final MultipleTelemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    // abstract void configureCommands(final GamepadEx gamepad);
}

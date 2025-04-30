package org.firstinspires.ftc.teamcode.OTHER_TEAMS_STUFF.subsystems.Subsytems.teleops

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.robotcore.external.Telemetry

class PandaTelemetryPacket(val telemetry: Telemetry) : TelemetryPacket() {
    override fun put(key: String?, value: Any?) {
        super.put(key, value)
        telemetry.addData(key, value)
    }
}
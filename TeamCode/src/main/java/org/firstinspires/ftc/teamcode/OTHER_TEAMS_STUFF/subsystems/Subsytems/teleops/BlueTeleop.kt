package org.firstinspires.ftc.teamcode.OTHER_TEAMS_STUFF.subsystems.Subsytems.teleops

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "CompetitionTeleop")
class BlueTeleop : CompetitionTeleop() {
    override val allianceColor: AllianceColor = AllianceColor.BLUE
}
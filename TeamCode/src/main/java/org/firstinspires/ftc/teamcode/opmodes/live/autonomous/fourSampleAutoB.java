package org.firstinspires.ftc.teamcode.opmodes.live.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.AutoConst;
import org.firstinspires.ftc.teamcode.constants.LEDConst;
import org.firstinspires.ftc.teamcode.opmodes.live.LiveAutoBase;

@Autonomous(name = "R_0+4+sub", group = "autonomous", preselectTeleOp = "Teleop Live")
public class fourSampleAutoB extends LiveAutoBase {

    AutoSample auto;
    @Override
    public void on_init() {
        auto = new AutoSample(robot, 0.75, 0.5);
        robot.led_control.alliance = LEDConst.BLUE;
        auto.sampleInit();

    }

    @Override
    public void on_start() {
        auto.highBasketFirst();
        auto.sampleReachIntake(AutoConst.SAMPLE_RIGHT);
        auto.highBasket();
        auto.sampleReachIntake(AutoConst.SAMPLE_MID);
        auto.highBasket();
        auto.sampleReachIntake(AutoConst.SAMPLE_LEFT);
        auto.highBasket();
        auto.subPark();
    }

    @Override
    public void on_stop() {
        auto.sampleStop();
    }
}
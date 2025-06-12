package org.firstinspires.ftc.teamcode.constants;

import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;

public class AutoConst {

    public static double gridMod = Math.PI * 1/2;

    //// Pose list ////
    public static Pose leftInitPose         = new Pose(32.5, 8.5, 0 + gridMod);
    public static Pose rightInitPose        = new Pose(85,  8.5, Math.PI * 1/2 + gridMod);

    public static Pose subPark              = new Pose(50,   60,  Math.PI + gridMod);
    public static Pose humanParkLeft            = new Pose(110,  10,  Math.PI * 1/2 + gridMod);
    public static Pose humanParkRight            = new Pose(132,  10,  Math.PI * 1/2 + gridMod);


    //// Sample Poses ////
    public static Pose highBasketPose       = new Pose(14,   14,  Math.PI * 1/4 + gridMod);
    public static Pose highBasketPoseOffset = new Pose(20,   20,  Math.PI * 1/4 + gridMod);

    public static Pose lowBasketPose        = new Pose(15,   15,  Math.PI * 1/4 + gridMod);

    public static int SAMPLE_RIGHT              = 1;
    public static Pose rightSamplePose          = new Pose(24, 30,  Math.PI * 1/2 + gridMod);
    public static Pose rightSamplePoseOffset    = new Pose(24, 15,  Math.PI * 1/2 + gridMod);

    public static int SAMPLE_MID                = 2;
    public static Pose midSamplePose            = new Pose(13, 30,  Math.PI * 1/2 + gridMod);
    public static Pose midSamplePoseOffset      = new Pose(13, 20,  Math.PI * 1/2 + gridMod);

    public static int SAMPLE_LEFT               = 3;
    public static Pose leftSamplePose           = new Pose(22,   44,  Math.PI + gridMod);
    public static Pose leftSamplePoseOffset     = new Pose(26,   44,  Math.PI + gridMod);

    public static int SAMPLE_PARTNER            = 4;
    public static Pose partnerSamplePose        = new Pose(55,   15,  0 + gridMod);
    public static Pose partnerSamplePoseOffset  = new Pose(47,   15,  0 + gridMod);

    //// Specimen Poses ////

    public static Pose leftSpecimenPose  = new Pose(116, 56, Math.PI * 3/2 + gridMod);
    public static Pose midSpecimenPose  = new Pose(124, 56, Math.PI * 3/2 + gridMod);
    public static Pose rightSpecimenPose  = new Pose(132, 56, Math.PI * 3/2 + gridMod);

}

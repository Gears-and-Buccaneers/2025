package frc.robot.util;

public enum Level {
    // wrist position -0.086, 8.375 in from the reef
    L1(0, 0x3cdb4e),
    L2(15.6, 0xd04242),
    L3(28, 0x40ccd0),
    L4(46, 0xecdb33);

    public final double height;
    public final int color;

    private Level(double height, int color){
        this.height = height;
        this.color = color;
    }

}

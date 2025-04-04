package frc.robot.util;

public enum Level {
    // wrist position -0.086, 8.375 in from the reef
    L1(0, 0xff0000),
    L2(17, 0xffff00),
    L3(28, 0x00ff00),
    L4(46, 0x0000ff);

    private static final Level[] lvls = values();

    public final double height;
    public final int color;

    private Level(double height, int color){
        this.height = height;
        this.color = color;
    }

    public Level next() {
        return lvls[Math.min(ordinal() + 1, lvls.length - 1)];
    }

    public Level prev() {
        return lvls[Math.max(ordinal() - 1, 0)];
    }
}

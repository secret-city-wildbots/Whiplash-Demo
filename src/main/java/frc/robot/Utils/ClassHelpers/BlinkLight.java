package frc.robot.Utils.ClassHelpers;

import edu.wpi.first.wpilibj.Timer;

public class BlinkLight {
    private double period;

    public BlinkLight(double period) {
        this.period = period;
    }
    /**
     * Returns a periodically toggled true or false where the period is period
     * @param period How frequently to switch between true and false
     * @return
     */
    public boolean blinkLight() {
        return ((Timer.getTimestamp() % period) * 2) < period;
    }
}

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import frc.robot.Utils.LEDHelpers;

public class LED {

    private final AddressableLED m_led = new AddressableLED(9);
    private final AddressableLEDBuffer m_ledBuffer;
    private AddressableLEDBuffer priorLedBuffer;
    private final int numberOfLEDs;
    enum LEDStates {
        NORMAL,
        PARTY,
        LOCATE,
        CHOOSEHUE
    }

    private double chaserStatus = 2;

    /**
     * Creates a new LED object to control LED outputs
     */
    public LED() {
        numberOfLEDs = 20;
        m_ledBuffer = new AddressableLEDBuffer(numberOfLEDs);
        priorLedBuffer = new AddressableLEDBuffer(numberOfLEDs);
    }



    /**
     * Updates the LED state based on driver inputs and robot states
     * @param driverController)
     */
    public void updateLED() {

        // remember previous LED buffer
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            priorLedBuffer.setRGB(i, m_ledBuffer.getRed(i), m_ledBuffer.getGreen(i), m_ledBuffer.getBlue(i));
        }

        // Depending on current LED state, send a new output to the LEDs
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {

            var partyGrbVal = LEDHelpers.rgbtogrb(LEDHelpers.hsvToRgb(i * 20 + (int) (chaserStatus), 1, 1));

            // Sets the specified LED to the HSV values
            m_ledBuffer.setRGB(i, (i * 20) + Integer.parseInt(partyGrbVal.substring(0, 2)),
                    (i * 20) + Integer.parseInt(partyGrbVal.substring(2, 4)),
                    (i * 20) + Integer.parseInt(partyGrbVal.substring(4, 6)));
        }
        // Increment chaser (Chaser is a double so that increments don't have to be integers)
        chaserStatus += 2;
        chaserStatus %= 360;
    }

    public void updateOutputs() {
        if (m_ledBuffer != priorLedBuffer) {
            m_led.setData(m_ledBuffer);
        }
    }
}

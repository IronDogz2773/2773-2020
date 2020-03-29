package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;

/**
 * Represents a group of ultrasonic sensors and their related components
 * @author Ilya Dzialendzik
 * @author Yury Dzialendzik
 */
public class ProximitySensor {
    private I2C proximity = new I2C(I2C.Port.kOnboard, 0x31);

    /**
     * Creates a new ProximitySensor
     */
    public ProximitySensor() {

    }

    /**
     * Gets distance readings from all ultrasonic sensors
     * @return A double array representing the distance reading of each sensor accordingly
     */
    public double[] getDistances() {
        double[] results = new double[4];

        if (Constants.enableProximitySensor) {
            byte[] buffer = new byte[8];
            proximity.read(0, 8, buffer);

            results[0] = (buffer[0] & 0xff) | ((buffer[1] & 0xff) << 8);
            results[1] = (buffer[2] & 0xff) | ((buffer[3] & 0xff) << 8);
            results[2] = (buffer[4] & 0xff) | ((buffer[5] & 0xff) << 8);
            results[3] = (buffer[6] & 0xff) | ((buffer[7] & 0xff) << 8);
        } else {
            final double FARAWAY = 10000;
            results[0] = FARAWAY;
            results[1] = FARAWAY;
            results[2] = FARAWAY;
            results[3] = FARAWAY;
        }

        return results;
    }
}
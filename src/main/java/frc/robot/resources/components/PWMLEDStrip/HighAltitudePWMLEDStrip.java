// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources.components.PWMLEDStrip;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.resources.math.Math;

/** Add your docs here. */
public class HighAltitudePWMLEDStrip {
    AddressableLED addressableLED;
    AddressableLEDBuffer buffer;
    int port;
    int length;

    // FOR RAINBOW AND SOLID CYCLE
    int firstPixelData;

    // FOR FIRE ANIMATION
    int cooling, sparking, cooldown, heat[];

    int peak1 = 0;
    int counter = 0;
    int dotCount1 = 0;
    int PEAK_FALL1 = 10;

    public HighAltitudePWMLEDStrip(int port, int length) {
        this.port = port;
        this.length = length;

        addressableLED = new AddressableLED(port);
        addressableLED.setLength(length);
        buffer = new AddressableLEDBuffer(length);

        addressableLED.setData(buffer);
        addressableLED.start();

        // FOR RAINBOW AND SOLID CYCLE
        firstPixelData = 0;

        // FOR FIRE ANIMATION
        heat = new int[length];
        for (int i = 0; i < heat.length; i++) {
            heat[i] = 255;
        }

    }

    /**
     * Turns off all lights in the LED strip. Calls for this method should
     * preferrably be instantaneous.
     */

    public void allLedsOff() {
        setSolidRGB(0, 0, 0);
    }

    /**
     * Sets a {@link TecbotLEDStrip} to a single solid color.
     * Please make sure that the parameters are in the following range:
     * 
     * @param hue        The HUE of the color 0-180
     * @param saturation The SATURATION of the color 0-255
     * @param value      The VALUE of the color 0-255
     */
    public void setSolidHSV(int hue, int saturation, int value) {

        hue = (int) Math.clamp(hue, 0, 180);
        saturation = (int) Math.clamp(saturation, 0, 255);
        value = (int) Math.clamp(value, 0, 255);

        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, hue, saturation, value);
        }
        addressableLED.setData(buffer);
    }

    /**
     * Sets a {@link TecbotLEDStrip} to a single solid color.
     * Please make sure that the parameters are in the following range:
     * 
     * @param r Red value of the color 0-255
     * @param g Green value of the color 0-255
     * @param b Blue value of the color 0-255
     */
    public void setSolidRGB(int r, int g, int b) {

        r = (int) Math.clamp(r, 0, 255);
        g = (int) Math.clamp(g, 0, 255);
        b = (int) Math.clamp(b, 0, 255);

        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
        addressableLED.setData(buffer);
    }

    /**
     * Sets the LED strip to a rainbow cycle animation.
     */

    public void setRainbowCycle() {
        // For every pixel
        for (var i = 0; i < buffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (firstPixelData + (i * 180 / buffer.getLength())) % 180;
            // Set the value
            buffer.setHSV(i, hue, 255, 255);
        }
        // Increase by to make the rainbow "move"
        firstPixelData += 3;
        // Check bounds
        firstPixelData %= 180;
        addressableLED.setData(buffer);
    }

    /**
     * Starts a fire animation of a color given its hue and saturation.
     * 
     * @param hue        hue of the color 0-180
     * @param saturation saturation of the color 0-255
     * @param cooling    How much the fire cools down. The lower the value, the more
     *                   intense the fire is. Recommended values: 10 for dim fire, 0
     *                   for intense fire.
     * @param sparking   How many sparks are randomly generated. The larger the
     *                   value, the more sparks are created. Recommended values: 10
     *                   for dim fire, 210 for intense fire.
     */
    public void setBasicFire(int hue, int saturation, int cooling, int sparking) {
        // Step 0. Check for mistakes from 8th layer and ensure parameters are alright
        hue = (int) Math.clamp(hue, 0, 180);
        saturation = (int) Math.clamp(saturation, 0, 255);
        cooling = (int) Math.clamp(cooling, 0, 255);
        sparking = (int) Math.clamp(sparking, 0, 255);

        // Step 1. Cool down every cell a little
        for (int i = 0; i < buffer.getLength(); i++) {
            cooldown = Math.randomInt(0, ((cooling * 10) / buffer.getLength()) + 2);
            if (cooldown > heat[i] || heat[i] > 255) {
                heat[i] %= 255;
            } else {
                heat[i] -= cooldown;
            }
        }

        // Step 2. Heat from each cell drifts 'up' and diffuses a little
        for (int k = buffer.getLength() - 1; k >= 2; k--) {
            heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
        }

        heat[1] = (heat[1] + 2 * heat[0]) / 3;

        // Step 3. Randomly ignite new 'sparks' near the bottom
        if (Math.randomInt(0, 255) < sparking) {
            int y = Math.randomInt(0, 7);

            heat[y] += Math.randomInt(160, 255);
        }

        // Step 4. Convert heat to LED colors
        for (int j = 0; j < buffer.getLength(); j++) {
            buffer.setHSV(j, hue, saturation, heat[j]);
        }

        addressableLED.setData(buffer);
    }

    /**
     * Starts a fire animation of a color given its hue, varying the fire's
     * intensity based on the value of an input within in a range. The higher the
     * input value within the range, the more intense the fire is i.e. in a scale
     * from 0 to 5, an input of 5 will produce a very intense fire while an input of
     * 0 will make the fire very dim.
     * 
     * @param hue      Hue of the fire's color.
     * @param input    Input that will impact the intensity of the fire.
     * @param min      The input's range minimum value.
     * @param max      The input's range maximum value.
     * @param length   Length of the fire in natural numbers 1..n. (Should
     *                 preferrably be equal to the length of the LED strip).
     * @param offset   How many LEDs should be skipped before the first LED of the
     *                 animation? The offset of the animation should be given in
     *                 natural numbers.
     * @param inverted Should the animation be upside down? (i.e. is the LED strip
     *                 oriented upside down?).
     */
    public void setFireWithVariableIntensity(int hue, double input, double min, double max, int length, int offset,
            boolean inverted) {
        // Step 0. Check for 8th layer mistakes and ensures parameters are alright
        hue = (int) Math.clamp(hue, 0, 180);
        double absoluteMagnitude = Math.clamp(input / (max - min), 0, 1);
        cooling = 255 - (int) (absoluteMagnitude * 255);

        // Change the coefficent of absoluteMagnitude to 100 for 100% flame intensity
        // when full joystick power. Turn down for less intensity in full joystick
        // power.
        sparking = (int) (absoluteMagnitude * 50);

        // Step 1. Cool down every cell a little
        // The second parameter in cooldown definition is changeable

        for (int i = 0; i < length; i++) {
            cooldown = Math.randomInt(0, ((cooling * 10) / length * 2));
            if (cooldown > heat[i] || heat[i] > 255) {
                heat[i] %= 255;
            } else {
                heat[i] -= cooldown;
            }
        }

        // Step 2. Heat from each cell drifts 'up' and diffuses a little
        // The diffusing is also somewhat arbitrary and changeable

        for (int k = length - 1; k >= 2; k--) {
            heat[k] = (heat[k - 1] + 2 * heat[k - 2]) / 3;
        }
        heat[1] = (heat[1] + 2 * heat[0]) / 3;

        // Step 3. Randomly ignite new 'sparks' near the bottom
        // randomLed picks one in the bottom to be a new spark. Change the second
        // parameter of its definition to increase or decrease the length of the
        // 'bottom'.
        // Currently, the 'bottom' is a fifth of the strip. Now a sixth haha

        // sparkIntensity non-surprisingly defines the intensity of the new spark
        // created in the randomLed
        // The clamp prevents it from going over, because if the value of randomLed is
        // something like 258, then the displayed intensity will be 258%255 = 3

        int randomLed = Math.randomInt(0, length / 6);
        int sparkIntensity = Math.randomInt(58 + sparking, 255);
        heat[randomLed] = (int) Math.clamp(heat[randomLed] + sparkIntensity, 0, 255);

        // Step 4. Convert heat to LED colors
        // Write parameters to buffer. The saturation parameter is changeable.

        for (int j = 0; j < length; j++) {
            if (!inverted)
                buffer.setHSV(j + offset, hue, 255 - (heat[j] / 3), (int) (heat[j] * 0.5));
            else {
                // System.out.println(offset + length - 1 - j);
                buffer.setHSV(offset + length - 1 - j, hue, 255 - (heat[j] / 3), (int) (heat[j] * 0.5));
            }
        }

        addressableLED.setData(buffer);
    }

    public int getLength() {
        return buffer.getLength();
    }

    /**
     * Sets the LEDs to a "sonidero" animation.
     * 
     * @param hue       the hue you want the sonidero to be
     * @param intensity the intensity (from -1 to 1 or 0-1) you want the noise to
     *                  have. This can be variable
     */
    public void setRhythmSingleHue(int hue, double intensity) {
        intensity = map(Math.abs(intensity), 0, 1, 0.08, 1);
        int baseline = (int) map(intensity, 0, 1, 1024.0 * 0.25, 1024.0 * 0.5);

        int sample = baseline
                + (int) map(Math.sin(counter * 0.3), -1, 1, 0, 1024 * intensity * intensity)
                + Math.randomInt(0, (int) (1024.0 * intensity * 0.5));

        int led = (int) map(sample, 0, 1024, 0, getLength() - 1); // max - min = peak-peak amplitude
        for (int i = 0; i < led; i++) {
            buffer.setHSV(
                    i,
                    hue,
                    255 - (int) map(i, 0, getLength() - 1, 0, 255 * 0.25),
                    255 - (int) map(i, 0, getLength() - 1, 0, 255 * 0.25));
        }

        for (int i = getLength() - 1; i > led; i--) {
            buffer.setHSV(i, 0, 0, 0);
        }

        if (led > peak1)
            peak1 = led; // Keep 'peak' dot at top
        if (peak1 > 1 && peak1 <= getLength() - 1)
            buffer.setHSV(peak1, 0, 0, 255);

        addressableLED.setData(buffer);

        // Every few frames, make the peak pixel drop by 1:

        if (++dotCount1 >= PEAK_FALL1) { // fall rate

            if (peak1 > 0)
                peak1--;
            dotCount1 = 0;
        }
        counter++;
    }

    /**
     * Sets the LEDs to a "sonidero" animation.
     * 
     * @param hue       the hue you want the sonidero to be
     * @param intensity the intensity (from -1 to 1 or 0-1) you want the noise to
     *                  have. This can be variable
     */
    public void setRhythmSingleHueDouble(int hue, double intensity, int length, int offset, boolean inverted) {
        intensity = map(Math.abs(intensity), 0, 1, 0.08, 1);
        int baseline = (int) map(intensity, 0, 1, 1024.0 * 0.25, 1024.0 * 0.5);

        int sample = baseline
                + (int) map(Math.sin(counter * 0.3), -1, 1, 0, 1024 * intensity * intensity)
                + Math.randomInt(0, (int) (1024.0 * intensity * 0.5));

        int led = (int) map(sample, 0, 1024, 0, length - 1); // max - min = peak-peak amplitude
        if (!inverted) {
            for (int i = 0; i < led; i++) {
                buffer.setHSV(
                        i + offset,
                        hue,
                        255 - (int) map(i, 0, length - 1, 0, 255 * 0.25),
                        255 - (int) map(i, 0, length - 1, 0, 255 * 0.25));
            }

            for (int i = getLength() - 1; i > led; i--) {
                buffer.setHSV(i + offset, 0, 0, 0);
            }

            if (led > peak1)
                peak1 = led; // Keep 'peak' dot at top
            if (peak1 > 1 && peak1 <= getLength() - 1)
                buffer.setHSV(peak1 + offset, 0, 0, 255);
        } else {
            for (int i = 0; i < led; i++) {
                buffer.setHSV(
                        offset + length - 1 - i,
                        hue,
                        255 - (int) map(i, 0, length - 1, 0, 255 * 0.25),
                        255 - (int) map(i, 0, length - 1, 0, 255 * 0.25));
            }

            for (int i = getLength() - 1; i > led; i--) {
                buffer.setHSV(offset + length - 1 - i, 0, 0, 0);
            }

            if (led > peak1)
                peak1 = led; // Keep 'peak' dot at top
            if (peak1 > 1 && peak1 <= getLength() - 1)
                buffer.setHSV(offset + length - 1 - peak1, 0, 0, 255);
        }
        addressableLED.setData(buffer);

        // Every few frames, make the peak pixel drop by 1:

        if (++dotCount1 >= PEAK_FALL1) { // fall rate

            if (peak1 > 0)
                peak1--;
            dotCount1 = 0;
        }
        counter++;
    }

    /**
     * Sets the LEDs to a "sonidero" animation.
     * 
     * @param intensity the intensity (from -1 to 1 or 0-1) you want the noise to
     *                  have. This can be variable
     */
    public void setRhythmColorLvls(double intensity) {
        intensity = map(Math.abs(intensity), 0, 1, 0.08, 1);
        int baseline = (int) map(intensity, 0, 1, 1024.0 * 0.25, 1024.0 * 0.5);

        int sample = baseline
                + (int) map(Math.sin(counter * 0.3), -1, 1, 0, 1024 * intensity * intensity)
                + Math.randomInt(0, (int) (1024.0 * intensity * 0.5));

        int led = (int) map(sample, 0, 1024, 0, getLength() - 1); // max - min = peak-peak amplitude

        for (int i = 0; i < led; i++) {
            int hue = 60 - (int) map(i, 0, getLength() - 1, 0, 60);
            buffer.setHSV(
                    i,
                    hue,
                    255,
                    255);
        }

        for (int i = getLength() - 1; i > led; i--) {
            buffer.setHSV(i, 0, 0, 0);
        }

        if (led > peak1)
            peak1 = led; // Keep 'peak' dot at top
        if (peak1 > 1 && peak1 <= getLength() - 1)
            buffer.setHSV(peak1, 0, 0, 255);

        addressableLED.setData(buffer);

        // Every few frames, make the peak pixel drop by 1:

        if (++dotCount1 >= PEAK_FALL1) { // fall rate

            if (peak1 > 0)
                peak1--;
            dotCount1 = 0;
        }
        counter++;
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return Math.clamp((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max);
    }
}
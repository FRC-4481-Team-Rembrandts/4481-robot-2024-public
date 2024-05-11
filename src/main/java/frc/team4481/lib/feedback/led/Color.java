package frc.team4481.lib.feedback.led;

/**
 * Class for representing colors in RGB and HSV color modes.
 * Allows for quick conversion between the two modes.
 * All values are in ranges supported by the WPILib {@code AddressableLED} class.
 * RGB values are in the range [0-255].
 * HSV values are in the range [0-180), [0-255], [0-255].
 * Intended to be used with {@code PWMLEDController} and {@code LEDStrip} classes.
 *
 * @see <a href="https://en.wikipedia.org/wiki/HSL_and_HSV">HSV on Wikipedia</a>
 * @see edu.wpi.first.wpilibj.AddressableLED
 * @see frc.team4481.lib.feedback.led.PWMLEDController
 * @see frc.team4481.lib.feedback.led.LEDStrip
 */
public class Color {
    private final RGB rgb;
    private final HSV hsv;

    /**
     * Creates a new color from HSV values.
     * @param hue The hue of the color [0-180).
     * @param saturation The saturation of the color [0-255].
     * @param value The value of the color [0-255].
     */
    public Color(int hue, int saturation, int value) {
        hsv = new HSV(hue, saturation, value);
        rgb = HSVtoRGB(hue, saturation, value);
    }

    /**
     * Creates a new color from RGB values.
     * @param red The red value of the color [0-255].
     * @param green The green value of the color [0-255].
     * @param blue The blue value of the color [0-255].
     */
    @SuppressWarnings("unused")
    public static Color fromRGB(int red, int green, int blue) {
        HSV hsv = RGBtoHSV(red, green, blue);
        return new Color(hsv.hue(), hsv.saturation(), hsv.value());
    }

    /**
     * Creates a new color from a hex string.
     * @param hex The hex string to convert (without #).
     */
    @SuppressWarnings("unused")
    public static Color fromHex(String hex) throws InvalidColorException {
        if (hex.length() != 6) {
            throw new InvalidColorException("Hex string must be 6 characters long.");
        }

        return fromRGB(
            Integer.valueOf(hex.substring(0, 2), 16),
            Integer.valueOf(hex.substring(2, 4), 16),
            Integer.valueOf(hex.substring(4, 6), 16)
        );
    }

    /**
     * Gets the RGB values of the color.
     * @return The RGB values of the color.
     */
    @SuppressWarnings("unused")
    public RGB getRGB() {
        return rgb;
    }

    /**
     * Gets the HSV values of the color.
     * @return The HSV values of the color.
     */
    @SuppressWarnings("unused")
    public HSV getHSV() {
        return hsv;
    }

    /**
     * Converts an HSV color to RGB.
     * @param h The hue of the color [0-180).
     * @param s The saturation of the color [0-255].
     * @param v The value of the color [0-255].
     */
    private RGB HSVtoRGB(int h, int s, int v) {
        if (s == 0) {
            return new RGB(v, v, v);
        }

        // The below algorithm is copied from Color.fromHSV and moved here for
        // performance reasons.

        // Loosely based on
        // https://en.wikipedia.org/wiki/HSL_and_HSV#HSV_to_RGB
        // The hue range is split into 60 degree regions where in each region there
        // is one rgb component at a low value (m), one at a high value (v) and one
        // that changes (X) from low to high (X+m) or high to low (v-X)

        // Difference between highest and lowest value of any rgb component
        final int chroma = (s * v) / 255;

        // Because hue is 0-180 rather than 0-360 use 30 not 60
        final int region = (h / 30) % 6;

        // Remainder converted from 0-30 to 0-255
        final int remainder = (int) Math.round((h % 30) * (255 / 30.0));

        // Value of the lowest rgb component
        final int m = v - chroma;

        // Goes from 0 to chroma as hue increases
        final int X = (chroma * remainder) >> 8;

        return switch (region) {
            case 0 -> new RGB(v, X + m, m);
            case 1 -> new RGB(v - X, v, m);
            case 2 -> new RGB(m, v, X + m);
            case 3 -> new RGB(m, v - X, v);
            case 4 -> new RGB(X + m, m, v);
            default -> new RGB(v, m, v - X);
        };
    }

    /**
     * Converts an RGB color to HSV.
     * @param r The red value of the color [0-255].
     * @param g The green value of the color [0-255].
     * @param b The blue value of the color [0-255].
     */
    private static HSV RGBtoHSV(int r, int g, int b) {
        double red = r / 255.0;
        double green = g / 255.0;
        double blue = b / 255.0;

        double cMax = Math.max(red, Math.max(green, blue));
        double cMin = Math.min(red, Math.min(green, blue));

        double delta = cMax - cMin;

        // Hue
        int hue;

        if (delta == 0) {
            hue = 0;
        } else if (cMax == red) {
            hue = (int) Math.round(60 * (((green - blue) / delta) % 6));
        } else if (cMax == green) {
            hue = (int) Math.round(60 * (((blue - red) / delta) + 2));
        } else {
            hue = (int) Math.round(60 * (((red - green) / delta) + 4));
        }

        // Saturation
        double saturation = (cMax == 0) ? 0 : delta / cMax;

        // Convert final values to correct range
        return new HSV(
                hue / 2,
                (int) Math.round(saturation * 255),
                (int) Math.round(cMax * 255)
        );
    }

    /**
     * Represents a color in RGB.
     * With red, green, and blue in the range [0-255].
     */
    public static class RGB {
        private final int red;
        private final int green;
        private final int blue;

        /**
         * Creates a new RGB color.
         * @param red The red value of the color [0-255].
         * @param green The green value of the color [0-255].
         * @param blue The blue value of the color [0-255].
         */
        public RGB(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }

        /**
         * Gets the red value of the color.
         * @return The red value of the color [0-255].
         */
        @SuppressWarnings("unused")
        public int red() {
            return red;
        }

        /**
         * Gets the green value of the color.
         * @return The green value of the color [0-255].
         */
        @SuppressWarnings("unused")
        public int green() {
            return green;
        }

        /**
         * Gets the blue value of the color.
         * @return The blue value of the color [0-255].
         */
        @SuppressWarnings("unused")
        public int blue() {
            return blue;
        }
    }

    /**
     * Represents a color in HSV.
     * With hue in the range [0-180),
     * saturation in the range [0-255],
     * and value in the range [0-255].
     */
    public static class HSV {
        private final int hue;
        private final int saturation;
        private final int value;

        /**
         * Creates a new HSV color.
         * @param hue The hue of the color [0-180).
         * @param saturation The saturation of the color [0-255].
         * @param value The value of the color [0-255].
         */
        public HSV(int hue, int saturation, int value) {
            this.hue = hue;
            this.saturation = saturation;
            this.value = value;
        }

        /**
         * Gets the hue of the color.
         * @return The hue of the color [0-180).
         */
        public int hue() {
            return hue;
        }

        /**
         * Gets the saturation of the color.
         * @return The saturation of the color [0-255].
         */
        public int saturation() {
            return saturation;
        }

        /**
         * Gets the value of the color.
         * @return The value of the color [0-255].
         */
        public int value() {
            return value;
        }
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }

        if (obj.getClass() != this.getClass()) {
            return false;
        }

        final Color other = (Color) obj;

        return this.rgb.red == other.rgb.red
            && this.rgb.green == other.rgb.green
            && this.rgb.blue == other.rgb.blue;
    }
}

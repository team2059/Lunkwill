package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class ButtonBox extends GenericHID {

    public ButtonBox(final int port) {
        super(port);
    }

    public enum Button {
        kGridTopLeft(1),
        kGridTopCenter(2),
        kGridTopRight(3),
        kGridMiddleLeft(4),
        kGridMiddleCenter(5),
        kGridMiddleRight(6),
        kGridBottomLeft(7),
        kGridBottomCenter(8),
        kGridBottomRight(9),
        // TODO: determine exact use for buttons 10-16
        // note: buttonbox has 20 inputs but driverstation only allows a max of 16
        kButton10(10), 
        kButton11(11), 
        kButton12(12),
        kButton13(13),
        kButton14(14),
        kButton15(15),
        kButton16(16);

        public final int value;

        Button(int value) {
            this.value = value;
        }
    }
}
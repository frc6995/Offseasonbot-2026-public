package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;

public final class ArrayUtil {
    private ArrayUtil() {}

    public static BaseStatusSignal[] concat(BaseStatusSignal[]... arrays) {
        int length = 0;
        for (BaseStatusSignal[] array : arrays) {
            length += array.length;
        }

        BaseStatusSignal[] combined = new BaseStatusSignal[length];
        int offset = 0;
        for (BaseStatusSignal[] array : arrays) {
            System.arraycopy(array, 0, combined, offset, array.length);
            offset += array.length;
        }
        return combined;
    }
}

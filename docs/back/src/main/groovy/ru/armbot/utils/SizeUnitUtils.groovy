package ru.armbot.utils

import ru.armbot.domain.SizeUnit
import java.math.RoundingMode

class SizeUnitUtils {

    static double fromM(String size, SizeUnit unit = SizeUnit.MM) {
        switch (unit) {
            case SizeUnit.MM:
                return new BigDecimal(size).setScale(2, RoundingMode.HALF_UP).doubleValue()
            default:
                return 0
        }
    }
}

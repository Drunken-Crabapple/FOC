#include "QD4310.h"

_Static_assert(sizeof(QD4310_StorageStatus) == sizeof(uint8_t),
               "QD4310_StorageStatus must match original C++ enum StorageStatus:uint8_t");
_Static_assert(QD4310_STORAGE_ALL_OK ==
                   (QD4310_STORAGE_BASE_CALIBRATE_OK |
                    QD4310_STORAGE_ANTICOGGING_CALIBRATE_OK |
                    QD4310_STORAGE_PID_PARAMETER_OK |
                    QD4310_STORAGE_LIMIT_OK |
                    QD4310_STORAGE_PLUG_OK |
                    QD4310_STORAGE_ZERO_POS_OK),
               "QD4310_STORAGE_ALL_OK must match original C++ STORAGE_ALL_OK mask");

int main(void) {
    return 0;
}

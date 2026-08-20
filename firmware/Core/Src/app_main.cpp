#include "iis3dwb_platform.h"
#include "iis3dwb_reg.h"

extern "C" void alt_main(void) {
    // 1. Link your SPI functions into dev_ctx
    IIS3DWB_Init_Platform();

    // 2. Call an ST driver function to verify full linking
    uint8_t dummy_id = 0;
    iis3dwb_device_id_get(&dev_ctx, &dummy_id);

    while (1) {
        // Main loop placeholder
    }
}

#ifndef __APP_H__
#define __APP_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef  void (*p_jump_func)(void);

#define NORMAL_START_ADDRESS			APP_START_ADDR  // Reference Makefile line 31, 32
#define BOOT_VER		"0.0.1"

enum fw_update_mode_t {
    FW_MODE_APPLICATION = 0,
    
    FW_MODE_BOOTLOADER_READY = 1, 
    
    FW_MODE_FLASHING = 2,
};
extern const char* boot_version;

extern int boot_main();

#ifdef __cplusplus
}
#endif

#endif //__APP_H__

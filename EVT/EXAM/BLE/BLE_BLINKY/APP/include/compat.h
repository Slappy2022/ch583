#ifndef __COMPAT_H
#define __COMPAT_H

#ifdef __cplusplus
extern "C" {
#endif

void compat_LedInit(void);
void compat_LedToggle(void);
void compat_CH58X_BLEInit(void);
void compat_HAL_TimeInit(void);
void compat_SetSysClock(void);

#ifdef __cplusplus
}
#endif

#endif

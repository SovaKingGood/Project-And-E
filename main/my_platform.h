#ifndef MY_PLATFORM_H
#define MY_PLATFORM_H

#include <uni.h>

/* Get the custom Bluepad32 platform */
struct uni_platform* get_my_platform(void);

/* Get number of connected controllers */
int my_platform_get_connected_controller_count(void);

/* Process controller data at controlled rate (called by timer) */
void my_platform_process_controller_data_timer(void);

#endif /* MY_PLATFORM_H */


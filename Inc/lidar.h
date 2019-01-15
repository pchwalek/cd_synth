#ifndef __lidar_H
#define __lidar_H

#include "cmsis_os.h"

#define VL53L1X 0x52
#define isInterrupt 0 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

osSemaphoreId lidarSampleReadySemaphoreHandle;

void activateLidar(void);
void deactivateLidar(void);
void LidarMeasurement(void);
void setupLidar(void);
void lidarStop(void);
void lidarStart(void);
void VL53L1_clear_int_and_enable_range(void);
void VL53L1_read_meas(void);
void activateLidarLED(void);
void deactivateLidarLED(void);
void genPOV_Map(int16_t* measurement);
uint32_t get_lidar_POV_map(void);

#endif

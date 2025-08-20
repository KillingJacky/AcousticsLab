/**
 * @file lorawan.h
 * @author Jack
 * @date 2025-08-15
 * @brief This file contains declarations and interfaces related to LoRaWAN functionality
 * for the AcousticsLab components.
 */
#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

void lorawan_init(const char* eui, const char* app_eui, const char* app_key, const char* freq);

/**
 * @brief Enqueues data to be sent over LoRaWAN.
 * 
 * The implementation of this function has a throttle mechanism to ensure that
 * the anomaly event is sent at a controlled rate, which is once per 30 seconds.
 *
 * @param abnormal Indicates if the event is abnormal.
 */
void lorawan_enqueue(bool abnormal);



#ifdef __cplusplus
}
#endif
#ifndef APP_H
#define APP_H

#include "platform/platform.h"

/**
 * Setup the application. Initialize all the required things
 *
 * If it errors, the board will go into FATAL_ERROR state and will be rebooted
 * by the watchdog
 * @return 0 if success, STATUS_X for errors
 */
int app_setup();

/**
 * Loop the application. Perform all periodic tasks here.
 *
 * This should do as little work as possible, as it blocks the platform tasks
 * (such as CAN communication).
 *
 * If it errors, the board will go into FATAL_ERROR state and will be rebooted
 * by the watchdog.
 * @return 0 if success, STATUS_X for errors
 */
int app_loop();

#endif
#include <Arduino.h>
#include "platform/log.h"

char printf_buffer[LOG_PRINTF_BUF_SIZE];
char irq_printf_buffer1[LOG_PRINTF_BUF_SIZE];
char irq_printf_buffer2[LOG_PRINTF_BUF_SIZE+7];
char irq_logbuf[LOG_IRQ_PRINTF_SIZE];

void afterirq_log() {
    if(irq_logbuf[0]) {
        SerialUSB.println("IRQ LOG: ");
        SerialUSB.println(irq_logbuf);
        irq_logbuf[0] = 0;
    }
}
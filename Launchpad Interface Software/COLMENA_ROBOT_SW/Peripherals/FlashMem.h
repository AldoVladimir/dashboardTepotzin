
#ifndef PERIPHERALS_FLASHMEM_H_
#define PERIPHERALS_FLASHMEM_H_

#include <stdint.h>
#include <third_party/spiffs/spiffs.h>
#include <third_party/spiffs/SPIFFSNVS.h>
#include "Board.h"
#include <ti/devices/cc26x0r2/driverlib/ssi.h>
#include <ti/drivers/GPIO.h>

/* SPIFFS configuration parameters */
#define SPIFFS_LOGICAL_BLOCK_SIZE    (4096)
#define SPIFFS_LOGICAL_PAGE_SIZE     (256)
#define SPIFFS_FILE_DESCRIPTOR_SIZE  (44)


bool MountFlashMemory();
bool UnmountFlashMemory();
bool WriteFlashMemory(char* fileName, uint8_t* data, uint8_t dataLen);
bool ReadFlashMemory(char* fileName, uint8_t* dataRead, uint8_t dataLen);
bool DeleteFile(char* fileName);
bool FormatFlashMemory();

void WakeUpExtFlash(void);
void ShutDownExtFlash(void);
void SendExtFlashByte(uint8_t byte);

#endif /* PERIPHERALS_FLASHMEM_H_ */

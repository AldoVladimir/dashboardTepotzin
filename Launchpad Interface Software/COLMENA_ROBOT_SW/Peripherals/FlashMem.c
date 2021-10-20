/*
 *  ======== Configuración tomada del ejemplo: spiffsexternal.c ========
 */

#include "FlashMem.h"

static uint8_t spiffsWorkBuffer[SPIFFS_LOGICAL_PAGE_SIZE * 2];              // SPIFFS needs RAM to perform operations on files.  It requires a work buffer which MUST be (2 * LOGICAL_PAGE_SIZE) bytes.
static uint8_t spiffsFileDescriptorCache[SPIFFS_FILE_DESCRIPTOR_SIZE * 4];  // The array below will be used by SPIFFS as a file descriptor cache.
static uint8_t spiffsReadWriteCache[SPIFFS_LOGICAL_PAGE_SIZE * 2];          // This array will be used by SPIFFS as a read/write cache.

spiffs         fs;
spiffs_file    fd;
spiffs_config  fsConfig;
SPIFFSNVS_Data spiffsnvsData;

bool MemoryMounted = false;


bool MountFlashMemory(){

    /*
     * Esta función monta la memoria flash para ser usada a través del puerto SPI0.
     * Si esta función no se llama antes de realizar alguna lectura, se producirá un error.
     */

    if(MemoryMounted){              // Si la memoria ya se había montado anteriormente, retorna True y no intenta montarla de nuevo.
        return true;
    }

    int32_t        status;

    WakeUpExtFlash();         // Le incida a la memoria que encienda, será accedida

    status = SPIFFSNVS_config(&spiffsnvsData, Board_NVSEXTERNAL, &fs, &fsConfig,
                              SPIFFS_LOGICAL_BLOCK_SIZE, SPIFFS_LOGICAL_PAGE_SIZE);

    if (status != SPIFFSNVS_STATUS_SUCCESS)
        return false;

    status = SPIFFS_mount(&fs, &fsConfig, spiffsWorkBuffer,
                          spiffsFileDescriptorCache, sizeof(spiffsFileDescriptorCache),
                          spiffsReadWriteCache, sizeof(spiffsReadWriteCache), NULL);

    if (status != SPIFFS_OK){
        /*
         * If SPIFFS_ERR_NOT_A_FS is returned; it means there is no existing
         * file system in memory.  In this case we must unmount, format &
         * re-mount the new file system.
         */
        if (status == SPIFFS_ERR_NOT_A_FS) {
            // File system not found; creating new SPIFFS fs...
            // Si se generó un error al intentar montar la memoria externa (error por falta de sistema de archivos)

            SPIFFS_unmount(&fs);                // Se desmonta la memoria
            status = SPIFFS_format(&fs);        // Se formatea la memoria
            if (status != SPIFFS_OK){           // Si se produce un error de formateo
                MemoryMounted = false;          // La memoria no se pudo montar correctamente
                return false;
            }

            // Se intenta montar la memoria nuevamente
            status = SPIFFS_mount(&fs, &fsConfig, spiffsWorkBuffer,
                                  spiffsFileDescriptorCache, sizeof(spiffsFileDescriptorCache),
                                  spiffsReadWriteCache, sizeof(spiffsReadWriteCache), NULL);

            if (status != SPIFFS_OK){           // Si se generó un error a la hora de montar
                MemoryMounted = false;          // La memoria no se pudo montar correctamente
                return false;
            }
        }
    }

    MemoryMounted = true;                       // La memoria se montó correctamente
    return true;
}

bool UnmountFlashMemory(){

    /*
     * Esta función desmonta la memoria flash y libera el puerto SSI0
     */

    if(MemoryMounted){                              // Si la memoria ya había sido previamente montada
        SPIFFS_unmount(&fs);                        // Se desmonta la memoria
        NVS_close(spiffsnvsData.nvsHandle);         // Se libera el puerto SPI0 para poder ser usado por la bobina Tx
        ShutDownExtFlash();                         // Pone a la memoria en modo de Deep Power Down (bajo consumo)
        MemoryMounted = false;                      // La memoria se desmontó correctamente
    }

    return true;
}

bool WriteFlashMemory(char* fileName, uint8_t* data, uint8_t dataLen){

    /*
     * Esta función guarda una cantidad de datos en un archivo guardado en la memoria flash externa.
     * Si el archivo no existe, lo crea y genera guarda los datos.
     * Recibe como parámetros @fileName (nombre del archivo a escribir), @dataRead (el apuntador a
     * un arreglo donde se leen los datos) y @dataLen (la cantidad de datos a leer del archivo).
     *
     * NOTA: Esta función asume que la memoria ya está montada.
     */

    fd = SPIFFS_open(&fs, fileName, SPIFFS_RDWR, 0);                            // Inteta abrir el arhivo

    if (fd < 0) {                                                               // Si el archivo no se pudo abrir
        fd = SPIFFS_open(&fs, fileName, SPIFFS_CREAT | SPIFFS_RDWR, 0);         // Se crea el archivo con el nombre especificado
        if (fd < 0){                                                            // Si aun no se puede crear el archivo
            return false;
        }
    }

    if (SPIFFS_write(&fs, fd, (void *) data, dataLen) < 0) {                    // Se intenta generar la esctritura del mensaje en el archivo
        SPIFFS_close(&fs, fd);                                                  // Cierra el archivo
        return false;                                                           // Si no se pudo escribir, entra a un loop infinito
    }

    SPIFFS_close(&fs, fd);                                                      // Después de escribir en el archivo, se cierra
    return true;
}

bool ReadFlashMemory(char* fileName, uint8_t* dataRead, uint8_t dataLen){

    /*
     * Esta función lee un archivo guardado en la memoria flash externa.
     * Recibe como parámetros @fileName (nombre del archivo), @dataRead
     * (el apuntador a un arreglo donde se leen los datos) y @dataLen (la
     * cantidad de datos a leer del archivo).
     *
     * NOTA: Esta función asume que la memoria ya está montada.
     */

    fd = SPIFFS_open(&fs, fileName, SPIFFS_RDWR, 0);                            // Inteta abrir el arhivo

    if (fd >= 0) {                                                              // Si se pudo abrir el archivo
        if (SPIFFS_read(&fs, fd, dataRead, dataLen) < 0) {                      // Se intenta leer el archivo. Se guarda en dataRead
            return false;                                                       // Error leyendo el archivo
        }
        SPIFFS_close(&fs, fd);                                                  // Se cierra el archivo
    }
    else{
        return false;                                                           // Error abriendo el archivo
    }
    return true;                                                                // Lectura correcta
}

bool DeleteFile(char* fileName){

    /*
     * Esta función borra un archivo específico de la memoria del robot.
     *
     * NOTA: Esta función asume que la memoria ya está montada.
     */

    fd = SPIFFS_open(&fs, fileName, SPIFFS_RDWR, 0);                        // Inteta abrir el arhivo

    if (fd >= 0) {                                                          // Si se pudo abrir el archivo
        SPIFFS_fremove(&fs, fd);                                            // Se intenta borrar el archivo
        SPIFFS_close(&fs, fd);                                              // Se cierra el archivo
    }
    else{
        return false;                                                       // El archivo no se pudo abrir
    }

    return true;                                                            // El archivo se eliminó correctamente
}

bool FormatFlashMemory(){

    /*
     * Esta función formatea y monta el sistema de archivos en
     * la memoria externa del robot.
     *
     * NOTA: Esta función asume que la memoria ya está montada.
     */

    int32_t        status;

    SPIFFS_unmount(&fs);                // Se desmonta la memoria
    status = SPIFFS_format(&fs);        // Se formatea la memoria
    if (status != SPIFFS_OK){           // Si se produce un error de formateo
        return false;
    }

    // Se intenta montar la memoria nuevamente
    status = SPIFFS_mount(&fs, &fsConfig, spiffsWorkBuffer,
                          spiffsFileDescriptorCache, sizeof(spiffsFileDescriptorCache),
                          spiffsReadWriteCache, sizeof(spiffsReadWriteCache), NULL);

    if (status != SPIFFS_OK){            // Si se generó un error a la hora de montar
        return false;
    }

    return true;
}

void WakeUpExtFlash(void){
    /*
     *  To wake up we need to toggle the chip select at
     *  least 20 ns and ten wait at least 35 us.
     */

    GPIO_setConfig(CC2640R2_LAUNCHXL_SPI_FLASH_CS, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH | GPIO_CFG_OUT_STR_MED);
    GPIO_setConfig(CC2640R2_LAUNCHXL_SPI0_MOSI,    GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW | GPIO_CFG_OUT_STR_MED);
    GPIO_setConfig(CC2640R2_LAUNCHXL_SPI0_CLK,     GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW | GPIO_CFG_OUT_STR_MED);


    GPIO_write(CC2640R2_LAUNCHXL_SPI_FLASH_CS, 0);          // Toggle chip select for ~20ns to wake ext. flash
    CPUdelay(1);                                            // 3 cycles per loop: 1 loop @ 48 Mhz ~= 62 ns

    GPIO_write(CC2640R2_LAUNCHXL_SPI_FLASH_CS, 1);
    CPUdelay(560);                                          // 3 cycles per loop: 560 loops @ 48 Mhz ~= 35 us

}

void ShutDownExtFlash(void){
    /*
     *  To be sure we are putting the flash into sleep and not waking it,
     *  we first have to make a wake up call
     */
    WakeUpExtFlash();

    uint8_t extFlashShutdown = 0xB9;

    SendExtFlashByte(extFlashShutdown);
}

void SendExtFlashByte(uint8_t byte){
    uint8_t i;

    GPIO_write(CC2640R2_LAUNCHXL_SPI_FLASH_CS, 0);

    for (i = 0; i < 8; i++) {
        GPIO_write(CC2640R2_LAUNCHXL_SPI0_CLK, 0);

        GPIO_write(CC2640R2_LAUNCHXL_SPI0_MOSI, (byte >> (7 - i)) & 0x01);
        GPIO_write(CC2640R2_LAUNCHXL_SPI0_CLK, 1);

        /*
         * Waste a few cycles to keep the CLK high for at
         * least 45% of the period.
         * 3 cycles per loop: 8 loops @ 48 Mhz = 0.5 us.
         */
        CPUdelay(8);
    }

    GPIO_write(CC2640R2_LAUNCHXL_SPI0_CLK, 0);
    GPIO_write(CC2640R2_LAUNCHXL_SPI_FLASH_CS, 1);

    /*
     * Keep CS high at least 40 us
     * 3 cycles per loop: 700 loops @ 48 Mhz ~= 44 us
     */
    CPUdelay(700);
}

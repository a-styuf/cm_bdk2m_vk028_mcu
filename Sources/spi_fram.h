#ifndef _SPI_FRAM_H_
#define _SPI_FRAM_H_

#include "main.h"
#include "spi.h"
#include "gpio.h"

#define SPI_NULL (void*)0

#define FRAM_USED_CHIP_NUMBER 4

typedef enum MEM_CHIP
{
  MEM0, MEM1, MEM2, MEM3,
  MEM4, MEM5,
  FRAM_MEM_NUMBER
} type_MEM_CH;

//определение настроек каждой отдельной памяти в соответствии с enum MEM_CHIP
#define MEM_CS_PORT {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA}
#define MEM_CS_LINE {3, 4, 5, 6, 7, 8}

#define FRAM_VOLUME_B (0x40000)
#define FRAM_FRAME_VOLUME_B (64)
#define FRAM_VOLUME_FRAMES (0x40000/64)


typedef struct
{
  type_SINGLE_GPIO cs[FRAM_MEM_NUMBER];
}type_FRAM;


int8_t fram_init(type_FRAM* fram_ptr);
void fram_write(type_FRAM* fram_ptr, uint8_t mem_num, uint32_t fram_addr, uint8_t *data_ptr, uint16_t leng);
void fram_read(type_FRAM* fram_ptr, uint8_t mem_num, uint32_t fram_addr, uint8_t *data_ptr, uint16_t leng);
int8_t fram_write_frame(type_FRAM* fram_ptr, uint32_t addr_fr, uint8_t *data_ptr);
int8_t fram_read_frame(type_FRAM* fram_ptr, uint32_t addr_fr, uint8_t *data_ptr);
#endif

#ifndef _MPP_H_
#define _MPP_H_

#include <string.h>
#include "main.h"
#include "internal_bus.h"
#include "task_planner.h"
#include "cyclogramma.h"
#include "frames.h"
#include "clock.h"
#include "mpp_conf.h"

// дефайны для переменных
#define MPP_REC_FIFO_DEPTH 4

#define MPP_DEFAULT_INTERVAL_MS (1000)

#define MPP_FORCE_START_PERIOD_TIMEOUT 1

#define MPP_EVENT_MEAS_INTERVAL_START (1 << 0)
#define MPP_EVENT_MEAS_INTERVAL_DATA_READY (1 << 1)

/// костыль для работы с отдельными каналами МПП
#define MPP_CHANNEL_REQUEST_SHIFT_MS (1000)

#define MPP_RP_BODY_ARRAY_SIZE        (12)

// значения по умолчанию для границ помеховых окон
#define MPP_WIN_BND_DEFAULT_1 0x01
#define MPP_WIN_BND_DEFAULT_2 0x08
#define MPP_WIN_BND_DEFAULT_3 0x20

/**
 * @brief  данная нумерация является общей для всей периферии и позволяет в общих списках понять смещение данного устройства
 * @note  данный список также используется для нумерации источников/обработчиков событий от/к периферии (необходимо следить, что бы количество источников не превышало количество доступных событий)
 */
typedef enum
{
  MPP_TYPE_MPP,
  MPP_TYPE_RP,
  MPP_TYPE_NUM
} type_MPP_TYPE;

//
#pragma pack(push, 2)

/**
 * @brief  структура помехи МПП
 */
typedef struct
{
  uint32_t AcqTime_s;  //+0
  uint32_t AcqTime_us; //+4
  uint32_t WidhtTime;  //+8
  uint16_t ZeroCount;  //+12
  uint16_t Peak;       //+14
  uint32_t Power;      //+16
  uint16_t Mean;       //+20
  uint16_t Noise;      //+22
} typeMPPRec;          // 24

/**
 * @brief  разобранная структура помехи МПП
 */
typedef struct
{
  float AcqTime_s;  //+0
  float AcqTime_us; //+4
  float WidhtTime;  //+8
  float ZeroCount;  //+12
  float Peak;       //+14
  float Power;      //+16
  float Mean;       //+20
  float Noise;      //+22
} typeMPPRecParc;   // 24


typedef struct
{
  uint32_t time;
  uint16_t amplitude;
} typeMPPMatrixVal;

/**
 * @brief значение оконных параметров канала МПП
 *
 */
typedef struct
{
  uint16_t bound[3];    //+0
  uint16_t num[3];      //+3
} typeMPPWindowChannel; // 6

/**
 * @brief  структура кадр МПП
 */
typedef union
{
  typeFrameStruct raw;
  struct
  {
    uint16_t header[5];
    uint16_t arch_count;
    uint16_t offset;
    typeMPPRec rec[2];
    uint16_t crc16;
  } mpp;
} typeMPPFrameUnion;

/**
 * @brief  структура помехи МПП
 */
typedef struct
{
  uint16_t time;
  uint16_t data;
} typeRP_Meas; // 4

/**
 * @brief  структура помехи МПП
 */
typedef struct
{
  typeRP_Meas meas[MPP_RP_BODY_ARRAY_SIZE];         //+0

} typeRP_Body;                  // 48

/**
 * @brief  структура кадр МПП
 */
typedef union
{
  typeFrameStruct raw;
  struct
  {
    uint16_t header[5];           //+0
    typeRP_Body rp_body;          //+10
    uint16_t changed_meas_interv; //+58
    uint16_t reserve;             //+60
    uint16_t crc16;
  } rp;
} typeRP_FrameUnion;

/**
 * @brief  структура управления МПП
 */
typedef struct
{
  // interfaces
  typeIBStruct *ib;
  // сfg
  uint16_t id;                        // id на внутренней шине
  uint16_t self_num;                  // номер устройства с точки зрения ЦМ
  uint16_t device_number, frame_type; // параметры прибора МПП, в котором он используется
  uint8_t channel;                    // канал МПП: 0 или 1
  uint16_t interval_ms;
  uint32_t *global_frame_num_ptr;
  float k, b; //калибровка для пересчета напряжения U[V] = k*ADC + b 
  //
  uint8_t mpp_type;
  uint8_t matrix_ena;
  //
  uint16_t pwr_off_bound, mode, const_mode, mpp_max_survey_ena; // данные установки
  uint16_t offset_to_set;
  float offs_float;
  uint32_t arch_cnt;
  uint8_t bound_to_set[3];
  // to task_planner
  uint8_t meas_event_num;
  uint64_t last_call_time_us;
  // rp data
  typeRP_FrameUnion rp_frame, mko_rp_frame;
  typeRP_Body rp_raw_body;
  uint32_t start_collect_data_time;
  uint8_t rp_raw_data_min_num;
  uint8_t rp_frame_data_ready; // флаг готовности данных в памяти на отправку в другой процесс
  // mpp data
  typeMPPFrameUnion frame, mko_frame;
  uint8_t frame_data_ready; // флаг готовности данных в памяти на отправку в другой процесс
  typeMPPRec rec_buff[MPP_REC_FIFO_DEPTH];
  typeMPPRecParc rec_buff_parced[MPP_REC_FIFO_DEPTH];
  uint8_t rec_ptr;
  typeMPPMatrixVal matrix_raw;
  typeMPPWindowChannel win_raw;
  // general
  uint8_t forced_start_flag;    // флаг необходимости принудительного запуска
  uint8_t forced_start_timeout; // таймаут на запуск принудительного старта
  uint8_t frame_pulse_cnt;      // количество считанных помех с последнего формирования кадра
  uint8_t arch_init_flag;    // флаг необходимости принудительного запуска
  //
  uint16_t current_meas_interval;  // данные для хранения измерительного интервала
  // cyclogram_ctrl
  typeCyclograma meas_cyclo, rp_cyclo;
} typeMPPStruct;

#pragma pack(pop)

//
void mpp_init(typeMPPStruct* mpp_ptr, 
              uint8_t self_num, 
              uint8_t id, 
              uint16_t device_number, 
              uint16_t frame_type, 
              uint8_t channel, 
              uint32_t offset, 
              typeIBStruct* ib_ptr, 
              uint32_t* gl_fr_num, 
              uint8_t mpp_type, 
              uint8_t mpp_matrix_ena);
void mpp_reset_parameters(typeMPPStruct *mpp_ptr);
//
int8_t mpp_process_tp(void *ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct *interface);
//
int8_t mpp_frame_forming(typeMPPStruct *mpp_ptr);
void mpp_mko_frame_forming(typeMPPStruct* mpp_ptr);
void mpp_set_calibr(typeMPPStruct* mpp_ptr, float k, float b);
void mpp_write_offset_to_set(typeMPPStruct* mpp_ptr, uint16_t offset_to_set);
//
void mpp_time_set(typeMPPStruct *mpp_ptr, uint32_t time_s);
void mpp_on_off(typeMPPStruct *mpp_ptr, uint32_t on_off);
void mpp_constant_mode(typeMPPStruct *mpp_ptr, uint32_t on_off);
void mpp_arch_mem_init(typeMPPStruct *mpp_ptr);
void mpp_set_offset(typeMPPStruct *mpp_ptr, uint16_t offset);
void mpp_pwr_off_bound_offset(typeMPPStruct *mpp_ptr, uint16_t bound);
//
void mpp_arch_count_offset_get(typeMPPStruct *mpp_ptr);
void mpp_forced_start(typeMPPStruct *mpp_ptr);
void mpp_relative_forced_start(typeMPPStruct *mpp_ptr);
void mpp_arch_mem_init_set_flag(typeMPPStruct* mpp_ptr);
//
void mpp_mtrx_max_val_get(typeMPPStruct *mpp_ptr);
void mpp_rp_mtrx_val_process(typeMPPStruct* mpp_ptr, uint16_t new_val);
void mpp_rp_frame_forming(typeMPPStruct* mpp_ptr);
void mpp_rp_mko_frame_forming(typeMPPStruct* mpp_ptr);
//
void mpp_win_bounds_set(typeMPPStruct *mpp_ptr, uint16_t bound_1, uint16_t bound_2, uint16_t bound_3);
void mpp_win_get(typeMPPStruct *mpp_ptr);
//
void mpp_struct_request(typeMPPStruct *mpp_ptr);
void mpp_struct_get(typeMPPStruct *mpp_ptr);
typeMPPRecParc mpp_parc_rec(typeMPPStruct* mpp_ptr, typeMPPRec rec);

// функции для работы циклограммы измерительного интервала
void mpp_meas_cycl_init(typeMPPStruct *mpp_ptr);
int32_t mpp_meas_cycl_on(void *ctrl_struct, uint8_t *data);
int32_t mpp_meas_cycl_arch_offset_get(void *ctrl_struct, uint8_t *data);
int32_t mpp_meas_cycl_struct_request(void *ctrl_struct, uint8_t *data);
int32_t mpp_meas_cycl_struct_get(void *ctrl_struct, uint8_t *data);
int32_t mpp_meas_cycl_forced_start(void *ctrl_struct, uint8_t *data);
int32_t mpp_meas_cycl_win_val_get(void *ctrl_struct, uint8_t *data);
int32_t mpp_meas_cycl_set_offset(void *ctrl_struct, uint8_t *data);
int32_t mpp_meas_cycl_set_win_bnd(void *ctrl_struct, uint8_t *data);
int32_t mpp_meas_cycl_rp_frame_forming(void* ctrl_struct, uint8_t* data);
int32_t mpp_meas_cycl_arch_init(void* ctrl_struct, uint8_t* data);
//
void __mpp_struct_rev(typeMPPRec *mpp_struct_ptr);
//
#endif

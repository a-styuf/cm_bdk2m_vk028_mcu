/**
 * @file stm.c
 * @author Алексей Стюф (a-styuf@yandex.ru)
 * @brief модуль управления сигналами СТМ для ОАИ ЦМ:
 *    - поддержка установки постоянных значений
 *    - поддержка установки временных значений с возвратам к постоянным
 * @version 0.1
 * @date 2024-02-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "stm.h"

/**
 * @brief инициализация модуля управления каналом СТМ
 * 
 * @param stm_ch_ptr указатель на структуру управления каналом СТМ
 * @param io_ptr указатель на порт GPIO 
 * @param value значение по умолчанию
 */
int8_t stm_ch_init(type_STM_Channel_Model *stm_ch_ptr, type_SINGLE_GPIO *io_ptr, uint8_t value)
{
  stm_ch_ptr->const_state = value;
  stm_ch_ptr->temporary_state = 0;
  stm_ch_ptr->temporary_timeout_ms = 0;
  stm_ch_ptr->io_ptr = io_ptr;
  //
	gpio_set(stm_ch_ptr->io_ptr, value&0x01);
  //
	return 0;
}

/**
 * @brief функция обработки состояния СТМ
 * 
 * @param stm_ch_ptr указатель на структуру управления модулем СТМ
 * @param period_ms время после прошлого вызова данной функции (должно быть больше 0 для корректной обработки состояния)
 */
void stm_ch_process(type_STM_Channel_Model *stm_ch_ptr, uint32_t period_ms)
{
  if (stm_ch_ptr->temporary_timeout_ms > 0){
    if (stm_ch_ptr->temporary_timeout_ms >= period_ms){
      stm_ch_ptr->temporary_timeout_ms -= period_ms;
    }
    else{
      stm_ch_ptr->temporary_timeout_ms = 0;
    }
    //
    stm_ch_ptr->current_state = stm_ch_ptr->temporary_state;
  }
  else{
    stm_ch_ptr->current_state = stm_ch_ptr->const_state;
  }
}

/**
 * @brief установка постоянного значения канала СТМ
 * 
 * @param stm_ch_ptr указатель на структуру управления модулем СТМ
 * @param val значение стм: 0-0, не 0 - 1
 */
void stm_ch_const_set(type_STM_Channel_Model *stm_ch_ptr, uint8_t val)
{
  stm_ch_ptr->const_state = val;
}

/**
 * @brief установка временного значения канала СТМ
 * 
 * @param stm_ch_ptr указатель на структуру управления модулем СТМ
 * @param val значение стм: 0-0, не 0 - 1
 * @param timeout_ms время выставление временного значения СТМ
 */
void stm_ch_temporary_set(type_STM_Channel_Model *stm_ch_ptr, uint8_t val, uint32_t timeout_ms)
{
  stm_ch_ptr->temporary_state = val;
  stm_ch_ptr->temporary_timeout_ms = timeout_ms;
}

/**
 * @brief запрос текущего значения СТМ
 * 
 * @param stm_ch_ptr указатель на структуру управления модулем СТМ
 * @return uint8_t 1 или 0
 */
uint8_t stm_ch_get_state(type_STM_Channel_Model *stm_ch_ptr)
{
  return stm_ch_ptr->current_state;
}

//Общее управление СТМ

void stm_init(type_STM_Model *stm_ptr, type_GPIO_OAI_cm* cm_io_ptr)
{
  uint8_t i=0;
  uint8_t stm_io_num[STM_NUM] = STM_CM_IO_NUM;
  uint8_t stm_default_val[STM_NUM] = STM_DEFAULT_VAL;
  //  
  stm_ptr->state = 0x00;
  stm_ptr->state_old = 0x00;
  stm_ptr->cm_io_ptr = cm_io_ptr;
  //
  for (i=0; i<STM_NUM; i++){
    stm_ch_init(&stm_ptr->ch[i], &stm_ptr->cm_io_ptr->stm[stm_io_num[i]], stm_default_val[i]);
  }
  //
  stm_ptr->call_interval_us = 0;
  stm_ptr->last_call_time_us = 0;
}

/**
  * @brief  обработка состояний телеметрии
	* @param  cm_ptr указатель на структуру управления
	* @param  time_us время МК us
  */
int8_t stm_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface)
{
	uint8_t retval=0;
	// uint16_t var_u16 = 0;
	type_STM_Model* stm_ptr = (type_STM_Model*)ctrl_struct;
	//
	if ((time_us - stm_ptr->last_call_time_us) > (STM_HANDLER_INTERVAL_MS*1000)) {
		stm_ptr->call_interval_us = time_us - stm_ptr->last_call_time_us;
		stm_ptr->last_call_time_us = time_us;
		// user code begin
		stm_process(stm_ptr, stm_ptr->call_interval_us/1000);
		// user code end
		retval = 1;
	}
	return retval;
}

void stm_process(type_STM_Model *stm_ptr, uint32_t period_ms)
{
  uint8_t i=0;
  for (i=0; i<STM_NUM; i++){
    stm_ch_process(&stm_ptr->ch[i], period_ms);
  }
	__stm_update_outputs(stm_ptr);
}

uint32_t __stm_update_state(type_STM_Model *stm_ptr)
{
  uint8_t i;
  uint32_t tmp_state = 0;
  //
  for (i=0; i<STM_NUM; i++){
    tmp_state |= ((stm_ptr->ch[i].current_state & 0x01) << i);
  }
  stm_ptr->state = tmp_state;
  //
  return stm_ptr->state;
}

void __stm_update_outputs(type_STM_Model *stm_ptr)
{
  uint8_t i = 0;
  //
  __stm_update_state(stm_ptr);
  //
  for (i=0; i<STM_NUM; i++){
    gpio_set(stm_ptr->ch[i].io_ptr, stm_ptr->ch[i].current_state);
  }
}

void stm_single_ch_const_set(type_STM_Model *stm_ptr, uint8_t num, uint32_t val)
{
  if (num < STM_NUM){
    stm_ch_const_set(&stm_ptr->ch[num], val & 0x01);
  }
}

void stm_single_ch_temporary_set(type_STM_Model *stm_ptr, uint8_t num, uint32_t val, uint32_t timeout_ms)
{
  if (num < STM_NUM){
    stm_ch_temporary_set(&stm_ptr->ch[num], val & 0x01, timeout_ms);
  }
}

void stm_const_set(type_STM_Model *stm_ptr, uint32_t val)
{
  uint8_t i=0;
  //
  for (i=0; i<STM_NUM; i++) {
    stm_ch_const_set(&stm_ptr->ch[i], val & (0x01 << i));
  }
}

void stm_temporary_set(type_STM_Model *stm_ptr, uint32_t val, uint32_t timeout_ms)
{
  uint8_t i=0;
  //
  for (i=0; i<STM_NUM; i++) {
    stm_ch_temporary_set(&stm_ptr->ch[i], (val>>i) & 0x01, timeout_ms);
  }
}

uint8_t stm_single_ch_get_state(type_STM_Model *stm_ptr, uint8_t num)
{
  if (num < STM_NUM){
    return stm_ch_get_state(&stm_ptr->ch[num]);
  }
  return 0;
}

uint32_t stm_get_state(type_STM_Model *stm_ptr)
{
  return __stm_update_state(stm_ptr);
}

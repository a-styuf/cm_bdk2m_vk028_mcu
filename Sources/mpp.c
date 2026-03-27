  /**
  ******************************************************************************
  * @file           : mpp.c
  * @version        : v1.0
  * @brief          : библиотека для управления отдельным каналом МПП, использует объект ВШ (internal_bus)
	* @note						: возможно использовать два отдельных канала в одном МПП как два независимых устройства (такой подход добавляет небольшой оверхэд, но объеденяет описание МПП с одним или несколькими каналами)
  * @author			    : Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  * @date						: 2021.09.22
  ******************************************************************************
  */

#include "mpp.h"

/**
  * @brief  инициализация структуры управления
	* @param  mpp_ptr указатель на структуру управления
	* @param  id номер устройства на внутренней шине
	* @param  device_number номер устройства в котором работает МПП
	* @param  frame_type тип кадра для МПП
	* @param  channel номер используемого канала МПП
	* @param  offset значение уставки определение помехи в кв АЦП
	* @param  ib_ptr указатель на внутреннюю шину
	* @param  gl_fr_num указатель на сквозной глобальный номер кадра
  */
void mpp_init(	typeMPPStruct* mpp_ptr, 
				uint8_t self_num, 
				uint8_t id, 
				uint16_t device_number, 
				uint16_t frame_type, 
				uint8_t channel, 
				uint32_t offset, 
				typeIBStruct* ib_ptr, 
				uint32_t* gl_fr_num, 
				uint8_t mpp_type, 
				uint8_t mpp_matrix_ena)
{
	mpp_reset_parameters(mpp_ptr);
	mpp_ptr->mpp_type = mpp_type;
	mpp_ptr->id = id;
	mpp_ptr->device_number = device_number;
	mpp_ptr->frame_type = frame_type;
	mpp_ptr->channel = channel;
	mpp_ptr->ib = ib_ptr;
	mpp_ptr->matrix_ena = mpp_matrix_ena;
	mpp_ptr->self_num = self_num;
	mpp_ptr->global_frame_num_ptr = gl_fr_num;
	// учтановка параметров по умолчанию
	mpp_ptr->offset_to_set = offset;
	mpp_ptr->bound_to_set[0] = MPP_WIN_BND_DEFAULT_1;
	mpp_ptr->bound_to_set[1] = MPP_WIN_BND_DEFAULT_2;
	mpp_ptr->bound_to_set[2] = MPP_WIN_BND_DEFAULT_3;
	// работы с циклограммами  МПП
	mpp_meas_cycl_init(mpp_ptr);
}

/**
  * @brief  инициализация структуры управления
	* @param  mpp_ptr указатель на структуру управления
  */
void mpp_reset_parameters(typeMPPStruct* mpp_ptr)
{
	//зануление данных на передачу в МКО и память
	memset((uint8_t*)mpp_ptr->rec_buff, 0xFE, sizeof(mpp_ptr->rec_buff));
	memset((uint8_t*)&mpp_ptr->frame, 0xFE, sizeof(mpp_ptr->frame));
	memset((uint8_t*)&mpp_ptr->mko_frame, 0xFE, sizeof(mpp_ptr->frame));
	memset((uint8_t*)&mpp_ptr->rp_frame, 0xFE, sizeof(mpp_ptr->frame));
	memset((uint8_t*)&mpp_ptr->mko_rp_frame, 0xFE, sizeof(mpp_ptr->frame));
	memset((uint8_t*)&mpp_ptr->rp_raw_body, 0x0, sizeof(mpp_ptr->rp_raw_body));
	//
	mpp_ptr->frame_data_ready = 0x00;
	mpp_ptr->rec_ptr = 0;
	//
	mpp_ptr->offset_to_set = 0; 
	mpp_ptr->offs_float = 0; 
	memset(mpp_ptr->bound_to_set, 0x00, sizeof(mpp_ptr->bound_to_set));
	mpp_ptr->pwr_off_bound = 0; 
	mpp_ptr->forced_start_flag = 0;
	mpp_ptr->forced_start_timeout = 0;
	mpp_ptr->frame_pulse_cnt = 0;
	mpp_ptr->last_call_time_us = 0;
	mpp_ptr->const_mode = 0;
	mpp_ptr->interval_ms = MPP_DEFAULT_INTERVAL_MS;
	mpp_ptr->meas_event_num = 0;
	mpp_ptr->start_collect_data_time = 0;
	mpp_ptr->arch_init_flag = 0;
	mpp_ptr->mpp_max_survey_ena = 0;
	//
	mpp_set_calibr(mpp_ptr, 1.0, 0.0);
}

/**
  * @brief  процесс обработки состояния МПП
	* @param  mpp_ptr указатель на структуру управления
	* @param  time_us время МК us
  */
int8_t mpp_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface)
{
	uint8_t retval = 0;
	typeMPPStruct* mpp_ptr = (typeMPPStruct*)ctrl_struct;
	/// запус обработчика по интервалу
	if ((time_us - mpp_ptr->last_call_time_us) > (mpp_ptr->interval_ms*1000)) {
		mpp_ptr->last_call_time_us = time_us;
		// user code begin
		mpp_ptr->current_meas_interval = *(uint16_t*)&interface->shared_mem[64];
		if(((cyclo_get_operation_status(&mpp_ptr->meas_cyclo) == 0) && (mpp_ptr->mpp_max_survey_ena)) && (mpp_ptr->matrix_ena)){
			mpp_mtrx_max_val_get(mpp_ptr);
			mpp_rp_mtrx_val_process(mpp_ptr, mpp_ptr->matrix_raw.amplitude);
		}
		// user code end
		retval = 1;
	}
	// обработка event-ов  //todo: сделать специальные фукнции обработки event-ов
	if (interface->event[mpp_ptr->self_num] & MPP_EVENT_MEAS_INTERVAL_START) 
	{
		interface->event[mpp_ptr->self_num] &= ~MPP_EVENT_MEAS_INTERVAL_START;
		cyclo_start(&mpp_ptr->meas_cyclo);
		mpp_ptr->mpp_max_survey_ena = 1;
		retval = 1;
	}
	else {
		//
	}
	//обработка циклограммы
	cyclo_handler(&mpp_ptr->meas_cyclo, time_us/1000);
	//обработка передачи данных
	if((mpp_ptr->frame_data_ready) && (mpp_ptr->mpp_type == MPP_TYPE_MPP))
	{
		mpp_ptr->frame_data_ready = 0;
		memcpy(interface->shared_mem, (uint8_t*)&mpp_ptr->mko_frame, sizeof(typeFrameStruct));
		interface->event[mpp_ptr->self_num] |= MPP_EVENT_MEAS_INTERVAL_DATA_READY;
	}
	else if((mpp_ptr->rp_frame_data_ready) && (mpp_ptr->mpp_type == MPP_TYPE_RP))
	{
		mpp_ptr->rp_frame_data_ready = 0;
		memcpy(interface->shared_mem, (uint8_t*)&mpp_ptr->mko_rp_frame, sizeof(typeFrameStruct));
		interface->event[mpp_ptr->self_num] |= MPP_EVENT_MEAS_INTERVAL_DATA_READY;
	}
	return retval;
}

void mpp_set_calibr(typeMPPStruct* mpp_ptr, float k, float b)
{
	mpp_ptr->k = k;
	mpp_ptr->b = b;
}

/**
  * @brief  установка переменной offset_to_set, которая будет прописана на следующем измерительном интервале
	* @param  mpp_ptr указатель на структуру управления
	* @param  offset уровень срабатывания в квантах АЦП
  */
void mpp_write_offset_to_set(typeMPPStruct* mpp_ptr, uint16_t offset_to_set)
{
	mpp_ptr->offset_to_set = offset_to_set;
}

/**
  * @brief  функция получения данных для складывания в кадр
	* @param  mpp_ptr указатель на структуру управления
	* @param  struct_num указатель на кадр
	* @retval  1 - кадр сформирован, 0 - недостаточно данных для кадра
  */
int8_t mpp_frame_forming(typeMPPStruct* mpp_ptr)
{
	if(mpp_ptr->rec_ptr >= 2){
		mpp_ptr->frame.raw.label = 0x0FF1;
		mpp_ptr->frame.raw.definer = frame_definer(0, mpp_ptr->device_number, NULL, mpp_ptr->frame_type);
		mpp_ptr->frame.raw.num = ((*mpp_ptr->global_frame_num_ptr)++)&0xFFFF;
		mpp_ptr->frame.raw.time = (clock_get_time_s());
		mpp_ptr->rec_ptr -= 1;
		mpp_ptr->frame.mpp.rec[0] = mpp_ptr->rec_buff[mpp_ptr->rec_ptr];
		mpp_ptr->rec_ptr -= 1;
		mpp_ptr->frame.mpp.rec[1] = mpp_ptr->rec_buff[mpp_ptr->rec_ptr];
		mpp_ptr->frame.raw.crc16 = frame_crc16((uint8_t*)&mpp_ptr->frame.raw, sizeof(typeFrameStruct) - 2);
		//
		mpp_mko_frame_forming(mpp_ptr);
		mpp_ptr->frame_data_ready = 1;
		return 1;
	}
	else{
		return 0;
	}
}

/**
  * @brief  функция формирования правильной байтовой раскладки кадра МПП
	* @param  mpp_ptr указатель на структуру управления
  */
void mpp_mko_frame_forming(typeMPPStruct* mpp_ptr)
{
	mpp_ptr->mko_frame.raw.label = mpp_ptr->frame.raw.label;
	mpp_ptr->mko_frame.raw.definer = mpp_ptr->frame.raw.definer;
	mpp_ptr->mko_frame.raw.num = mpp_ptr->frame.raw.num;
	mpp_ptr->mko_frame.raw.time = _rev_u32_by_u16(mpp_ptr->frame.raw.time);
	//
	mpp_ptr->mko_frame.mpp.arch_count = mpp_ptr->frame.mpp.arch_count;
	mpp_ptr->mko_frame.mpp.offset = mpp_ptr->frame.mpp.offset;
	//
	for(uint8_t rec_num = 0; rec_num<2; rec_num++){
		mpp_ptr->mko_frame.mpp.rec[rec_num].AcqTime_s	= _rev_u32_by_u16(mpp_ptr->frame.mpp.rec[rec_num].AcqTime_s);
		mpp_ptr->mko_frame.mpp.rec[rec_num].AcqTime_us	= _rev_u32_by_u16(mpp_ptr->frame.mpp.rec[rec_num].AcqTime_us);
		mpp_ptr->mko_frame.mpp.rec[rec_num].WidhtTime	= (mpp_ptr->frame.mpp.rec[rec_num].WidhtTime);
		mpp_ptr->mko_frame.mpp.rec[rec_num].ZeroCount	= (mpp_ptr->frame.mpp.rec[rec_num].ZeroCount);
		mpp_ptr->mko_frame.mpp.rec[rec_num].Peak		= (mpp_ptr->frame.mpp.rec[rec_num].Peak);
		mpp_ptr->mko_frame.mpp.rec[rec_num].Power		= _rev_u32_by_u16(mpp_ptr->frame.mpp.rec[rec_num].Power);
		mpp_ptr->mko_frame.mpp.rec[rec_num].Mean		= mpp_ptr->frame.mpp.rec[rec_num].Mean;
		mpp_ptr->mko_frame.mpp.rec[rec_num].Noise		= mpp_ptr->frame.mpp.rec[rec_num].Noise;
	}
	mpp_ptr->mko_frame.raw.crc16 = frame_crc16((uint8_t*)&mpp_ptr->mko_frame.raw, sizeof(typeFrameStruct) - 2);
}

/**
  * @brief  установка времени МПП (широковещательная)
	* @param  mpp_ptr указатель на структуру управления
	* @param  time_s время МК s
  */
void mpp_time_set(typeMPPStruct* mpp_ptr, uint32_t time_s)
{
	uint16_t data[2];
	data[0] = (time_s >> 16) & 0xFFFF;
	data[1] = (time_s >> 0) & 0xFFFF; 
	ib_run_transaction(mpp_ptr->ib, 0xFF, 111, 0, 2, data);
}

/**
  * @brief  включение отключение работы отдельного канала
	* @param  mpp_ptr указатель на структуру управления
	* @param  on_off 1 - включение, 0 - отключение
  */
void mpp_on_off(typeMPPStruct* mpp_ptr, uint32_t on_off)
{
	uint16_t data[2];
	mpp_ptr->mode = (on_off) ? 0x01 : 0x00;
	data[0] = __REV16((mpp_ptr->channel << 8) | (2));
	data[1] = __REV16(mpp_ptr->mode); 
	ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 16, 0, 2, data);
}

/**
  * @brief  включение отключение режима константа (широковещательная)
	* @param  mpp_ptr указатель на структуру управления
	* @param  on_off 1 - включение, 0 - отключение
  */
void mpp_constant_mode(typeMPPStruct* mpp_ptr, uint32_t on_off)
{
	uint16_t data[2];
	if(mpp_ptr->mpp_type == MPP_TYPE_MPP){
		mpp_ptr->const_mode = (on_off) ? 0xAAAA : 0x0000;
		data[0] = __REV16(mpp_ptr->const_mode);
		ib_run_transaction(mpp_ptr->ib, 0xFF, MB_F_CODE_106, 0x5555, 2, data);
	}
	else{
		mpp_ptr->const_mode = (on_off) ? 0x01 : 0x00;
		data[0] = __REV16((4 << 8) | (255));
		data[1] = __REV16(mpp_ptr->const_mode); 
		ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 16, 0, 2, data);
	}
}

/**
  * @brief  инициализация памяти
	* @param  mpp_ptr указатель на структуру управления
	*/
void mpp_arch_mem_init(typeMPPStruct* mpp_ptr)
{
	uint16_t data[2];
	data[0] = __REV16((mpp_ptr->channel << 8) | (84));
	data[1] = __REV16(0x0000); 
	ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 16, 0, 1, data);
}

/**
  * @brief  установка уровня срабатывания для МПП
	* @param  mpp_ptr указатель на структуру управления
	* @param  offset уровень срабатывания в квантах АЦП
  */
void mpp_set_offset(typeMPPStruct* mpp_ptr, uint16_t offset)
{
	uint16_t data[2];
	data[0] = __REV16((mpp_ptr->channel << 8) | (1));
	data[1] = __REV16(offset);
	ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 16, 0, 2, data);
}

/**
  * @brief  установка уровня отключения питания при превышении напряжения для МПП
	* @param  mpp_ptr указатель на структуру управления
	* @param  bound уровень срабатывания в квантах АЦП
  */
void mpp_pwr_off_bound_offset(typeMPPStruct* mpp_ptr, uint16_t bound)
{
	uint16_t data[2];
	mpp_ptr->pwr_off_bound = bound;
	data[0] = __REV16((mpp_ptr->channel << 8) | (90));
	data[1] = __REV16(mpp_ptr->pwr_off_bound);
	ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 16, 0, 2, data);
}

/**
  * @brief  запрос значений установки уровня срабатывания и количества помех в архивной памяти МПП
	* @param  mpp_ptr указатель на структуру управления
	* @param  bound уровень срабатывания в квантах АЦП
  */
void mpp_arch_count_offset_get(typeMPPStruct* mpp_ptr)
{
	uint8_t in_data[8], data_offset = 0;
	if (ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 3, 119, 4, NULL) > 0){
		data_offset = (mpp_ptr->channel == 0) ? 0 : 2;
		ib_get_answer_data(mpp_ptr->ib, in_data, 8);
		mpp_ptr->frame.mpp.arch_count = __REV16(*(uint16_t*)&in_data[0 + data_offset]);
		mpp_ptr->frame.mpp.offset = __REV16(*(uint16_t*)&in_data[4 + data_offset]);
		//
		mpp_ptr->offs_float = mpp_ptr->frame.mpp.offset * mpp_ptr->k;
		mpp_ptr->arch_cnt = mpp_ptr->frame.mpp.arch_count;
	}
	else{
		//
	}
}

/**
  * @brief  принудительный запуск регистрации МПП
	* @param  mpp_ptr указатель на структуру управления
  */
void mpp_forced_start(typeMPPStruct* mpp_ptr)
{
	uint16_t data[2];
	data[0] = __REV16((mpp_ptr->channel << 8) | (81));
	data[1] = __REV16(0x0001);
	ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 16, 0, 2, data);
}

/**
  * @brief  принудительный запуск регистрации МПП при наличии флага запуска
	* @param  mpp_ptr указатель на структуру управления
  */
void mpp_relative_forced_start(typeMPPStruct* mpp_ptr)
{
	if(mpp_ptr->forced_start_flag){
		mpp_ptr->forced_start_flag = 0;
		if (mpp_ptr->forced_start_timeout >= MPP_FORCE_START_PERIOD_TIMEOUT){
			mpp_ptr->forced_start_timeout = 0;
			mpp_forced_start(mpp_ptr);
		}
		else{
			mpp_ptr->forced_start_timeout += 1;
			mpp_on_off(mpp_ptr, 1);
		}
	} 
}

/**
 * @brief установка флага для форматирования архивной памяти при следующем измерительном интервале
 * 
 * @param mpp_ptr 
 */
void mpp_arch_mem_init_set_flag(typeMPPStruct* mpp_ptr)
{
	mpp_ptr->arch_init_flag = 1;
}

/**
 * @brief Получение данных максимумов амплитуды за последний интервал
 * 
 * @param mpp_ptr 
 */
void mpp_mtrx_max_val_get(typeMPPStruct* mpp_ptr)
{
	uint8_t in_data[8];
	uint16_t reg_addr = 0;
    //
    mpp_ptr->matrix_raw.time = clock_get_time_s();
    //
	reg_addr = (mpp_ptr->channel == 0) ? 128 : 129;
	if (ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 3, reg_addr, 1, NULL) > 0){
		ib_get_answer_data(mpp_ptr->ib, in_data, 2);
		mpp_ptr->matrix_raw.amplitude = __REV16(*(uint16_t*)&in_data[0]);
	}
	else{
		mpp_ptr->matrix_raw.amplitude  = 0xFEFE;
	}
}

/**
 * @brief Обработка максимальных значений МПП или РП за прошлую секунду для формирования данных для РП
 * 
 * @param mpp_ptr 
 */
void mpp_rp_mtrx_val_process(typeMPPStruct* mpp_ptr, uint16_t new_val)
{
	uint16_t rp_data=0xFFFF;
	//
	if (mpp_ptr->start_collect_data_time == 0) mpp_ptr->start_collect_data_time = clock_get_time_s();
    // нахождение минимального значение в буфере для хранения данных рп
    for(uint8_t meas_num = 0; meas_num < MPP_RP_BODY_ARRAY_SIZE; meas_num++){
		if(mpp_ptr->rp_raw_body.meas[meas_num].data < rp_data){
			rp_data = mpp_ptr->rp_raw_body.meas[meas_num].data;
			mpp_ptr->rp_raw_data_min_num = meas_num;
		}
	}
	// установка нового значения в минимальную величину, в случае, если величина больше этого минимального значения
	if(mpp_ptr->rp_raw_body.meas[mpp_ptr->rp_raw_data_min_num].data < new_val){
		mpp_ptr->rp_raw_body.meas[mpp_ptr->rp_raw_data_min_num].data = new_val;
		mpp_ptr->rp_raw_body.meas[mpp_ptr->rp_raw_data_min_num].time = clock_get_time_s() - mpp_ptr->start_collect_data_time;
	}
}


/**
 * @brief сброс параметров для подсчета данных РП и Матрицы
 * 
 * @param mpp_ptr 
 */
void mpp_rp_mtrx_val_reset(typeMPPStruct* mpp_ptr)
{
	mpp_ptr->start_collect_data_time = 0;
	memset((uint8_t *)&mpp_ptr->rp_raw_body, 0x00, sizeof(typeRP_Body));
	mpp_ptr->rp_raw_data_min_num  = 0;
}

/**
 * @brief Формирование кадра РП
 * 
 * @param mpp_ptr 
 */
void mpp_rp_frame_forming(typeMPPStruct* mpp_ptr)
{
	mpp_ptr->rp_frame.raw.label = 0x0FF1;
	mpp_ptr->rp_frame.raw.definer = frame_definer(0, mpp_ptr->device_number, NULL, mpp_ptr->frame_type);
	mpp_ptr->rp_frame.raw.num = ((*mpp_ptr->global_frame_num_ptr)++)&0xFFFF;
	mpp_ptr->rp_frame.raw.time = (clock_get_time_s());
	//
	memset(mpp_ptr->rp_frame.raw.data, 0xFE, sizeof(mpp_ptr->rp_frame.raw.data));
	//
	memcpy((uint8_t*)&mpp_ptr->rp_frame.rp.rp_body, (uint8_t*)&mpp_ptr->rp_raw_body, sizeof(typeRP_Body));
	mpp_ptr->rp_frame.rp.changed_meas_interv = mpp_ptr->current_meas_interval;
	//
	mpp_ptr->rp_frame.raw.crc16 = frame_crc16((uint8_t*)&mpp_ptr->rp_frame.raw, sizeof(typeFrameStruct) - 2);
	//
	mpp_rp_mko_frame_forming(mpp_ptr);
	//
	mpp_ptr->rp_frame_data_ready = 1;
}

/**
 * @brief Формирование кадра РП для передачи через МКО для учета нюансов по порядку байт
 * 
 * @param mpp_ptr 
 */
void mpp_rp_mko_frame_forming(typeMPPStruct* mpp_ptr)
{
	mpp_ptr->mko_rp_frame.raw.label = mpp_ptr->rp_frame.raw.label;
	mpp_ptr->mko_rp_frame.raw.definer = mpp_ptr->rp_frame.raw.definer;
	mpp_ptr->mko_rp_frame.raw.num = mpp_ptr->rp_frame.raw.num;
	mpp_ptr->mko_rp_frame.raw.time = _rev_u32_by_u16(mpp_ptr->rp_frame.raw.time);
	//
	memset(mpp_ptr->mko_rp_frame.raw.data, 0xFE, sizeof(mpp_ptr->rp_frame.raw.data));
	//
	memcpy((uint8_t*)&mpp_ptr->mko_rp_frame.rp.rp_body, (uint8_t*)&mpp_ptr->rp_frame.rp.rp_body, sizeof(typeRP_Body));
	mpp_ptr->mko_rp_frame.rp.changed_meas_interv = mpp_ptr->rp_frame.rp.changed_meas_interv;
	//
	mpp_ptr->mko_rp_frame.raw.crc16 = frame_crc16((uint8_t*)&mpp_ptr->mko_rp_frame.raw, sizeof(typeFrameStruct) - 2);

}

/**
 * @brief Установка границ окон для канала МПП
 * 
 * @param mpp_ptr 
 * @param bound_1 
 * @param bound_2 
 * @param bound_3 
 */
void mpp_win_bounds_set(typeMPPStruct* mpp_ptr, uint16_t bound_1, uint16_t bound_2, uint16_t bound_3)
{
	uint16_t data[4];
	data[0] = __REV16((mpp_ptr->channel << 8) + 0x10);
	data[1] = __REV16((bound_1 << 4) & 0xFFFF); 
	data[2] = __REV16((bound_2 << 4) & 0xFFFF); 
	data[3] = __REV16((bound_3 << 4) & 0xFFFF); 
	ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 16, 0, 4, data);
}

/**
 * @brief Запрос оконных значений  // TODO: необходимо уточнить у АА возможность сброса при обращении к определенным регистрам
 * 
 * @param mpp_ptr 
 */
void mpp_win_get(typeMPPStruct* mpp_ptr)
{
	uint8_t in_data[32];
	uint16_t reg_addr = 0;
    //
	reg_addr = (mpp_ptr->channel == 0) ? 130 : 133;
	if (ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 3, reg_addr, 9, NULL) > 0){
		ib_get_answer_data(mpp_ptr->ib, in_data, 24);
		mpp_ptr->win_raw.bound[0] 	= __REV16(*(uint16_t*)&in_data[0]);
		mpp_ptr->win_raw.bound[1] 	= __REV16(*(uint16_t*)&in_data[2]);
		mpp_ptr->win_raw.bound[2] 	= __REV16(*(uint16_t*)&in_data[4]);
		mpp_ptr->win_raw.num[0] 	= __REV16(*(uint16_t*)&in_data[12]);
		mpp_ptr->win_raw.num[1] 	= __REV16(*(uint16_t*)&in_data[14]);
		mpp_ptr->win_raw.num[2] 	= __REV16(*(uint16_t*)&in_data[16]);
	}
	else{
		memset((uint8_t *)&mpp_ptr->win_raw, 0xFE, sizeof(typeMPPWindowChannel));
	}
}


/**
  * @brief  запрос на подготовку 2х структур помех из архивной памяти МПП
	* @param  mpp_ptr указатель на структуру управления
  */
void mpp_struct_request(typeMPPStruct* mpp_ptr)
{
	uint16_t data[2];
	data[0] = __REV16((mpp_ptr->channel << 8) | (4));
	data[1] = __REV16(0x0001);
	ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 16, 0, 2, data);
}

/**
  * @brief  запрос к МПП на выдачу 2-х структур из архивной памяти в оперативную
	* @param  mpp_ptr указатель на структуру управления
  */
void mpp_struct_get(typeMPPStruct* mpp_ptr)
{
	uint8_t i, in_data[64];
	typeMPPRec rec[2];
	// todo: сделать запрос двух помех. Одна помеха из-за ошибки в модуле МПП, которая не дает читать две помехи за раз (подробности у А.А.Дорошкина)
	if (ib_run_transaction(mpp_ptr->ib, mpp_ptr->id, 3, 7, 13, NULL) > 0){
		ib_get_answer_data(mpp_ptr->ib, in_data, 26*1);
		// сохраняем полученные структуры в свои переменные, но отбрасываем номер канала
		memcpy((uint8_t*)&rec[0], in_data+2, sizeof(typeMPPRec));
		//memcpy((uint8_t*)&rec[1], in_data+2+24+2, sizeof(typeMPPRec));
		// приводим порядок байт к используемому в МК
		__mpp_struct_rev(&rec[0]);
		//__mpp_struct_rev(&rec[1]);
		//
		mpp_ptr->forced_start_flag = 1;
		//
		for (i=0; i<1; i++){
			if((rec[i].AcqTime_s != 0) || (rec[i].AcqTime_us != 0)){ // проверяем есть ли измерение в прочитанных данных
				mpp_ptr->forced_start_flag = 0; // если кадры были получены, то принудительный запуск МПП не нужен
				// заполняем буфер с помехами: если буфер переполнился пишем всегда в полседюю помеху пока буфер не будет вычитан
				// предполагаем, что такая ситуация маловероятна
				mpp_ptr->rec_buff[mpp_ptr->rec_ptr] = rec[i];
				mpp_ptr->rec_buff_parced[mpp_ptr->rec_ptr] = mpp_parc_rec(mpp_ptr, rec[i]);
				mpp_ptr->rec_ptr++;
				if (mpp_ptr->rec_ptr >= MPP_REC_FIFO_DEPTH) mpp_ptr->rec_ptr = MPP_REC_FIFO_DEPTH-1;
				if (mpp_ptr->rec_ptr >= 2) {
					mpp_frame_forming(mpp_ptr);
				}
			}
		}
	}
}

typeMPPRecParc mpp_parc_rec(typeMPPStruct* mpp_ptr, typeMPPRec rec)
{
	typeMPPRecParc rec_parced;
	rec_parced.AcqTime_s 	= rec.AcqTime_s;
	rec_parced.AcqTime_us 	= rec.AcqTime_us/1000000.;
	rec_parced.WidhtTime 	= rec.WidhtTime * 25E-9f;
	rec_parced.ZeroCount 	= rec.ZeroCount;
	rec_parced.Peak 		= rec.Peak * mpp_ptr->k;
	rec_parced.Power 		= rec.Power * mpp_ptr->k * 25E-9f;
	rec_parced.Mean 		= rec.Mean * mpp_ptr->k + mpp_ptr->b;
	rec_parced.Noise 		= rec.Noise * mpp_ptr->k;
	return rec_parced;
}

/**
  * @brief  перестановка байт в словах попарно в структуре МПП
	* @param  mpp_struct_ptr указатель на структуру
  */
void __mpp_struct_rev(typeMPPRec* mpp_struct_ptr)
{
	typeMPPRec rec = *mpp_struct_ptr;
	//
	rec.AcqTime_s 	= __REV(rec.AcqTime_s);
	rec.AcqTime_us 	= __REV(rec.AcqTime_us);
	rec.WidhtTime 	= __REV(rec.WidhtTime);
	rec.ZeroCount 	= __REV16(rec.ZeroCount);
	rec.Peak 		= __REV16(rec.Peak);
	rec.Power 		= __REV(rec.Power);
	rec.Mean 		= __REV16(rec.Mean);
	rec.Noise 		= __REV16(rec.Noise);
	//
	*mpp_struct_ptr = rec;
}

/**
  * @brief  инициализация циклограмм работы с МПП
	* @param  mpp_ptr указатель на структуру управления
  */
void mpp_meas_cycl_init(typeMPPStruct* mpp_ptr)
{
	uint32_t channel_delay;
	uint8_t data[32] = {0};
	// циклограмма инициализации МПП
	cyclo_init(&mpp_ptr->meas_cyclo, "mpp");
	// todo: продумать систему расшивки работы с каналами МПП. Возможно надо вернуться к идее работы с МПП в одном процессе для двух каналов
	channel_delay = (mpp_ptr->channel == 0) ? 0 : MPP_CHANNEL_REQUEST_SHIFT_MS;  // костыль из-за того, что во время подготовки данных первого канала, второй канал не может запрашивать данные из архива
	//
	cyclo_add_step(&mpp_ptr->meas_cyclo, CYCLO_DO_NOTHING, (void*)mpp_ptr, 50, 0 + channel_delay, data);
	cyclo_add_step(&mpp_ptr->meas_cyclo, mpp_meas_cycl_struct_request, (void*)mpp_ptr, 0, 150, data);
	cyclo_add_step(&mpp_ptr->meas_cyclo, mpp_meas_cycl_struct_get, (void*)mpp_ptr, 0, 50, data);
	cyclo_add_step(&mpp_ptr->meas_cyclo, mpp_meas_cycl_win_val_get, (void*)mpp_ptr, 0, 50, data);
	cyclo_add_step(&mpp_ptr->meas_cyclo, mpp_meas_cycl_forced_start, (void*)mpp_ptr, 0, 50, data);
	cyclo_add_step(&mpp_ptr->meas_cyclo, mpp_meas_cycl_set_offset, (void*)mpp_ptr, 0, 50, data);
	cyclo_add_step(&mpp_ptr->meas_cyclo, mpp_meas_cycl_set_win_bnd, (void*)mpp_ptr, 0, 50, data);
	cyclo_add_step(&mpp_ptr->meas_cyclo, mpp_meas_cycl_rp_frame_forming, (void*)mpp_ptr, 0, 0, data);
	cyclo_add_step(&mpp_ptr->meas_cyclo, mpp_meas_cycl_arch_offset_get, (void*)mpp_ptr, 50, 0, data);
	cyclo_add_step(&mpp_ptr->meas_cyclo, mpp_meas_cycl_arch_init, (void*)mpp_ptr, 0, 0, data);
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
int32_t mpp_meas_cycl_on(void* ctrl_struct, uint8_t* data)
{
	typeMPPStruct* mpp_ptr = (typeMPPStruct*) ctrl_struct;
	mpp_on_off(mpp_ptr, 1);
	return 0;
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
int32_t mpp_meas_cycl_arch_offset_get(void* ctrl_struct, uint8_t* data)
{
	typeMPPStruct* mpp_ptr = (typeMPPStruct*) ctrl_struct;
	mpp_arch_count_offset_get(mpp_ptr);
	return 0;
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
int32_t mpp_meas_cycl_struct_request(void* ctrl_struct, uint8_t* data)
{
	typeMPPStruct* mpp_ptr = (typeMPPStruct*) ctrl_struct;
	if (mpp_ptr->mpp_type == MPP_TYPE_MPP){
		mpp_struct_request(mpp_ptr);
	}
	return 0;
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
int32_t mpp_meas_cycl_struct_get(void* ctrl_struct, uint8_t* data)
{
	typeMPPStruct* mpp_ptr = (typeMPPStruct*) ctrl_struct;
	if (mpp_ptr->mpp_type == MPP_TYPE_MPP){
		mpp_struct_get(mpp_ptr);
	}
	return 0;
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
int32_t mpp_meas_cycl_forced_start(void* ctrl_struct, uint8_t* data)
{
	typeMPPStruct* mpp_ptr = (typeMPPStruct*) ctrl_struct;
	mpp_relative_forced_start(mpp_ptr);
	return 0;
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
int32_t mpp_meas_cycl_win_val_get(void* ctrl_struct, uint8_t* data)
{
	typeMPPStruct* mpp_ptr = (typeMPPStruct*) ctrl_struct;
	mpp_win_get(mpp_ptr);
	return 0;
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
int32_t mpp_meas_cycl_set_offset(void* ctrl_struct, uint8_t* data)
{
	typeMPPStruct* mpp_ptr = (typeMPPStruct*) ctrl_struct;
	mpp_set_offset(mpp_ptr, mpp_ptr->offset_to_set);
	return 0;
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
int32_t mpp_meas_cycl_set_win_bnd(void* ctrl_struct, uint8_t* data)
{
	typeMPPStruct* mpp_ptr = (typeMPPStruct*) ctrl_struct;
	mpp_win_bounds_set(mpp_ptr, mpp_ptr->bound_to_set[0], mpp_ptr->bound_to_set[1], mpp_ptr->bound_to_set[2]);
	return 0;
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
int32_t mpp_meas_cycl_rp_frame_forming(void* ctrl_struct, uint8_t* data)
{
	typeMPPStruct* mpp_ptr = (typeMPPStruct*) ctrl_struct;
	if (mpp_ptr->mpp_type == MPP_TYPE_RP){
		mpp_rp_frame_forming(mpp_ptr);
		mpp_rp_mtrx_val_reset(mpp_ptr);
	}
	return 0;
}

/**
  * @brief  обертка функция для согласования типов
	* @param  ctrl_struct указатель на структуру управления
  */
int32_t mpp_meas_cycl_arch_init(void* ctrl_struct, uint8_t* data)
{
	typeMPPStruct* mpp_ptr = (typeMPPStruct*) ctrl_struct;
	if(mpp_ptr->arch_init_flag){
		mpp_arch_mem_init(mpp_ptr);
		mpp_ptr->arch_init_flag = 0;
	}
	return 0;
}

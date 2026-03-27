#include "main.h"
#include "gpio.h"
#include "timers.h"
#include "clock.h"
#include "adc.h"
#include "mko_bc.h"
#include "mko_rt.h"
#include "wdt.h"
#include "frame_mem.h"
//
#include "retarget_conf.h"  // перенаправление printf в UART3
#include "task_planner.h"
#include "power_management.h"
#include "internal_bus.h"
#include "cm.h"
#include "dep.h"
#include "dir.h"
#include "bdd.h"
#include "ddii.h"
#include "mpp.h"

//***Общие настройки***//

//версия прошивки
#define CM_SW_VERSION 			  "0.18"
// номер устройства
#define FRAME_DEV_ID 			    218 // (218 - отработочный)
// параметры МКО
#define MKO_ADDRESS_DEFAULT 	0 // 0 - адрес берется с разъема, не 0 - адрес МКО (29 - по умолчанию)

//
extern uint32_t SystemCoreClock;  /*100_000_000*/
// создание объектов отдельных программных моделей 
type_GPIO_OAI_cm gpio;
typeADCStruct adc;
typeTPStruct tp;
typeCMModel cm; 
typePower pwr;
typeIBStruct ib;
type_STM_Model stm;
typeMKOBCStruct mko_bc;
typeMKORTStruct mko_rt;
//
typeMPPStruct mpp[MPP_DEV_NUM]; //! программная модель периферийных модулей МПП
typeDEPStruct dep;
typeDIRStruct dir;
typeBDDStruct bdd[2];
typeDDIIStruct ddii;

//
void SystemCoreClockUpdate(void);
//
int Countdown = 0;
uint8_t flag = 0;
char Str[256];

// прототипы функций для использования внутри main.c (описание в функциях)
void __main_process_registration_box(void);
void __main_init_peripheral_modules(void);
void __main_base_init(void);

/**
 * @brief суперцикл
 * правка только с согласованием с автором проекта
 * остальные правки на свой страх и риск
 * @return int никогда не возвращает, так как работает бесконечно
 */
int main() {
  // базовая инициализация
  SystemCoreClockUpdate();
  retarget_init();
  SysTick_Config(SystemCoreClock/1000);  // 1 ms  interrupt period
  //
  __main_base_init();
  //
	WDT_Init();
  //
  while(1) {
    tp_handler(&tp); // обработка планировщика задач
    WDRST;
  }/*while(1)*/  
}

/**
  * @brief  обертка для инициализации всей периферии
*/
void __main_init_peripheral_modules(void)
{
	// настройки каналов МПП
	uint32_t mpp_offsets_array[MPP_DEV_NUM] = MPP_DEF_OFFSET;
	uint32_t mpp_ib_id[MPP_DEV_NUM] = MPP_ID;
	uint32_t mpp_ch_num[MPP_DEV_NUM] = MPP_CHANNENUM_ID;
	uint8_t mpp_matrix_ena[MPP_DEV_NUM] = MPP_MATRIX_ENA;
  float mpp_ch_k[MPP_DEV_NUM] = MPP_CHAN_K;
	float mpp_ch_b[MPP_DEV_NUM] = MPP_CHAN_B;
	uint8_t uint8_var = 0;
	/**
	 * @brief  цикл используется для последовательной нумерации схожих устройств. Использование необязательно
	 */
	for (uint8_var = 0; uint8_var<MPP_DEV_NUM; uint8_var++){
		mpp_init(	&mpp[uint8_var], 
					MPP1 + uint8_var, 
					mpp_ib_id[uint8_var], 
					FRAME_DEV_ID, MPP1+uint8_var+1, 
					mpp_ch_num[uint8_var], 
					mpp_offsets_array[uint8_var], 
					cm.ib_ptr, 
					&cm.global_frame_num,
          MPP_TYPE_MPP,
          mpp_matrix_ena[uint8_var]
        );
    mpp_set_calibr(&mpp[uint8_var], mpp_ch_k[uint8_var], mpp_ch_b[uint8_var]);
	}
	dep_init(&dep, DEP, 10, FRAME_DEV_ID, DEP+1, cm.ib_ptr, &cm.global_frame_num);
	dir_init(&dir, DIR, 10, FRAME_DEV_ID, DIR+1, cm.ib_ptr, &cm.global_frame_num);
	bdd_init(&bdd[0], BDD1, 13, FRAME_DEV_ID, BDD1+1, cm.mko_bc_ptr, MKO_BC_BUS_A, &cm.global_frame_num);
	bdd_init(&bdd[1], BDD2, 14, FRAME_DEV_ID, BDD2+1, cm.mko_bc_ptr, MKO_BC_BUS_A, &cm.global_frame_num);
	ddii_init(&ddii, DDII, 15, FRAME_DEV_ID, DDII+1, cm.mko_bc_ptr, MKO_BC_BUS_A, &cm.global_frame_num);
}

// Командный интерфейс
/**
  * @brief  обертка для регистрации всех необходимых процессов
*/
void __main_process_registration_box(void)
{
	// процессы для поддержания работы ЦМ и его систем: питания, АЦП, внутренней шины
	tp_process_registration(&tp, &cm_process_tp, &cm, CM*64, TP_SHARED_MEM_VOL_B);
	tp_process_registration(&tp, &ib_process_tp, &ib, 0, 0);
	tp_process_registration(&tp, &adc_process_tp, &adc, 0, 0);
	tp_process_registration(&tp, &pwr_process_tp, &pwr, 0, 0);
  tp_process_registration(&tp, &stm_process_tp, &stm, 0, 0);
	// Процессы периферийных устройств
	for (uint8_t uint8_var = 0; uint8_var < MPP_DEV_NUM; uint8_var++){
		tp_process_registration(&tp, &mpp_process_tp, &mpp[uint8_var], 64*(MPP1+uint8_var), 64);
	}
	tp_process_registration(&tp, &dep_process_tp, &dep, 64*(DEP), 64);
	tp_process_registration(&tp, &dir_process_tp, &dir, 64*(DIR), 64);
	tp_process_registration(&tp, &bdd_process_tp, &bdd[0], 64*(BDD1), 64);
	tp_process_registration(&tp, &bdd_process_tp, &bdd[1], 64*(BDD2), 64);
	tp_process_registration(&tp, &ddii_process_tp, &ddii, 64*(DDII), 64);
}

/**
  * @brief  обертка для базовой инициализации БЭ через команду МКО (или ВШ)
*/
void __main_base_init(void)
{
	printf("\nCM Init: start\n");
	// отключение периферии
  
  //
  printf("SW v%s\n", CM_SW_VERSION);
	printf("\n______ \\_(o_O)_/ ______\n\n");
	printf("CM_BDK2M (1921VK028 based) frame_dev_id <%d>, mko_addr_default <%d> \n", FRAME_DEV_ID, MKO_ADDRESS_DEFAULT);
	//
	oai_cm_gpio_init(&gpio);
	clock_init(0);
	printf("%s: clock init \n", now());
	Timers_Init();
  clock_set_time_s(0);
  Timer_Delay(0, 100);
	printf("%s: clock time after 100 ms delay (OK - is about 0.1 s)\n", now());
	// инициализация структур
  printf("%s: gpio init\n", now());
  mko_bc_init(&mko_bc);
  printf("%s: mko_bc init\n", now());
  stm_init(&stm, &gpio);
  while (mko_rt_init(&mko_rt, MKO_ADDRESS_DEFAULT) == 0){
    Timer_Delay(1, 1000);
    WDRST;
    gpio_set(stm.ch[AMKO].io_ptr, (gpio_get(stm.ch[AMKO].io_ptr) & 0x01) ^ 0x01);
  }
  stm_single_ch_const_set(&stm, AMKO, 0);
  stm_single_ch_const_set(&stm, NKBE, 1);
  ib_init(&ib);
  printf("%s: internal bus init\n", now());
  printf("%s: stm init\n", now());
  adc_init(&adc);
  printf("%s: adc init\n", now());
  pwr_init(&pwr, &adc, &gpio); 
  printf("%s: pwr init\n", now());
  //
	cm_init(  &cm, 
            CM,
            CM_SELF_MB_ID,
            &mko_rt,
            &mko_bc,
            &ib,
            &pwr,
            &gpio.io[17],
            &stm,
            FRAME_DEV_ID,
            CM_SW_VERSION,
            CM+1);  // инициализация объекта программной модели ЦМ (включает в себя все периферийные модели ЦМ: ВШ, МКО, GPIO, ADC и т.п.)
	// включение периферии
	__main_init_peripheral_modules();
	// отключение планировщика задач
	tp_init(&tp);
	printf("%s: TP init done \n", now());
	//
	WDRST;
	// сброс времени
	clock_set_time_s(0);
	// регистрация процессов
	__main_process_registration_box();
}

void cm_mko_command_interface_handler(typeCMModel *cm_ptr)
{
  typeFrameStruct frame;
	uint16_t sa_data[32];
  uint16_t data_arr[32];
	uint16_t var_uint16 = 0;
	uint32_t var_uint32 = 0;
	uint8_t rt_status;
  uint8_t rt_sa;
  uint8_t ib_id;
  uint8_t ib_fcode;
  uint16_t ib_addr;
  uint16_t ib_len;
	//
  rt_status =  mko_rt_transaction_handler(&mko_rt, &rt_sa);
  if (rt_status) stm_single_ch_temporary_set(&stm, AMKO, 1, 10000);
  switch(rt_status){
    case 1: // запись в ПА для команд
      memcpy((uint8_t*)sa_data, (uint8_t*)mko_rt.sa_rx_data, 64);
      switch(rt_sa){
        case CM_MKO_SA_CMD:
          printf("%s: MKO cmd sa<%d> d<0x%04X %04X %04X %04X>\n", now(), rt_status, sa_data[0], sa_data[1], sa_data[2], sa_data[3]);
          switch(sa_data[0]){
            case (CMD_TEST):
              //
              break;
            case (CMD_SYNCH_TIME):
              cm_mko_cmd_synch_time(cm_ptr, sa_data[1], sa_data[2]);
              break;
            case (CMD_INIT):
              __main_base_init();
              // архивация памяти
              fr_mem_format(&cm.mem);
              fr_mem_set_rd_ptr(&cm.mem, 0);
              fr_mem_set_wr_ptr(&cm.mem, 0);
              cm.ctrl.rst_cnter = 0;
              cm.global_frame_num = 0;
              pwr_update_default_state(&pwr);
              break;
            case (CMD_SET_INTERVAL):
              cm_set_interval_value(cm_ptr, sa_data[1], sa_data[2]);
              break;
            case (CMD_SET_READ_PTR):
              switch(sa_data[1]){
                case 0: //CM
                  fr_mem_set_rd_ptr(&cm_ptr->mem, sa_data[2]);
                  break;
                case 1: //BDD1
                  bdd_set_rd_ptr(&bdd[0], sa_data[2]);
                  break;
                case 2: //BDD2
                  bdd_set_rd_ptr(&bdd[1], sa_data[2]);
                  break;
              }
              break;
            case (CMD_JUMP_TO_DEFENCE_AREA):
                fr_mem_set_rd_ptr_to_defense_area(&cm_ptr->mem);
              break;
            case (CMD_SET_MPP_OFFSET):
              if ((sa_data[1] >= 1) && (sa_data[1] <= MPP_DEV_NUM)) {
                mpp_write_offset_to_set(&mpp[sa_data[1] - 1], sa_data[2]);
              }
              break;
            case (CMD_CONST_MODE):
              cm_constant_mode_ena(&cm, sa_data[1] & 0x01);
              mpp_constant_mode(&mpp[0], sa_data[1] & 0x01); // команда широковещательная
              ddii_const_mode(&ddii, sa_data[1] & 0x01);
              bdd_constant_mode(&bdd[0], sa_data[1] & 0x01);
              bdd_constant_mode(&bdd[1], sa_data[1] & 0x01);
              dep_constant_mode(&dep, sa_data[1] & 0x01);
              dir_constant_mode(&dir, sa_data[1] & 0x01);
              break;
            case (CMD_CURRENT_LVL):
              if ((sa_data[1] < PWR_CH_NUMBER)){
                pwr_set_bound(cm.pwr_ptr, sa_data[1], sa_data[2]);
              }
              else {}
              break;
            case (CMD_PWR_CH_CTRL):
              if ((sa_data[1] < PWR_CH_NUMBER)){
                //pwr_on_off_by_num_with_ena(cm.pwr_ptr, sa_data[1], (uint8_t)(sa_data[2] & 0x01));
                pwr_queue_put_cmd(&pwr, 200, sa_data[1], (uint8_t)(sa_data[2] & 0x01), 0);
              }
              break;
            case (CMD_SPEEDY_MODE):
              cm_set_speedy_mode(cm_ptr, sa_data[1], sa_data[2]);
              break;
            case (CMD_CM_RESET):
              if (sa_data[1] == 0xA55A){
                printf("Rst by WDG ");
                Timer_Delay(1, 10000); // не более 11 сек
                printf("Error\n");
                NVIC_SystemReset();
              }
              break;
            default:
              break;
				}
          break;
        case CM_MKO_SA_ARCH_REQUEST_CM:
          if ((sa_data[0] == 0) && (sa_data[1] == 0) && (sa_data[2] == 0) && (sa_data[3] == 0) ){
            fr_mem_read_data_frame(&cm_ptr->mem, (uint8_t*)&frame);
            mko_rt_write_to_subaddr(&mko_rt, CM_MKO_SA_ARCH_READ_CM, (uint16_t*)&frame);
          }
          break;
        case CM_MKO_SA_TECH_CMD:
          memcpy((uint8_t*)sa_data, (uint8_t*)mko_rt.sa_rx_data, 64);
          switch(sa_data[0]){
            case (TCMD_CHECK_MIRROR):
              sa_data[0] |= 0x0100;
              mko_rt_write_to_subaddr(cm_ptr->mko_rt_ptr, CM_MKO_SA_TECH_CMD, (uint16_t*)sa_data);
              break;
            case (TCMD_CHECK_MEM):
              sa_data[0] |= 0x0100;
              var_uint32 =  fr_mem_check(&cm_ptr->mem);
              sa_data[1] = (var_uint32 >> 16) & 0xFFFF;
              sa_data[2] = (var_uint32 >> 0) & 0xFFFF;
              mko_rt_write_to_subaddr(cm_ptr->mko_rt_ptr, CM_MKO_SA_TECH_CMD, (uint16_t*)sa_data);
              break;
            case (TCMD_ANY_FRAME_READ):
              sa_data[0] |= 0x0100;
              var_uint16 = fr_mem_read_any_frame(&cm_ptr->mem, sa_data[1], (uint8_t *)&frame);
              sa_data[2] = (var_uint16 >> 0) & 0xFFFF;
              mko_rt_write_to_subaddr(cm_ptr->mko_rt_ptr, CM_MKO_SA_TECH_CMD, (uint16_t*)sa_data);
              mko_rt_write_to_subaddr(cm_ptr->mko_rt_ptr, CM_MKO_SA_TECH_RD, (uint16_t*)&frame);
              break;
            case (TCMD_SET_OPERATION_TIME):
              sa_data[0] |= 0x0100;
              if (sa_data[1] == 0xAFAF){
                cm_ptr->ctrl.operation_time = (sa_data[2] << 16) + (sa_data[3] << 0);
              }
              mko_rt_write_to_subaddr(cm_ptr->mko_rt_ptr, CM_MKO_SA_TECH_CMD, (uint16_t*)sa_data);
              break;
            case (TCMD_SET_STM):
              sa_data[0] |= 0x0100;
              stm_temporary_set(cm_ptr->stm_ptr, sa_data[1], sa_data[2]);
              mko_rt_write_to_subaddr(cm_ptr->mko_rt_ptr, CM_MKO_SA_TECH_CMD, (uint16_t*)sa_data);
              break;
            case (TCMD_SET_IB):
              sa_data[0] |= 0x0100;
              ib_id = (sa_data[1] >> 8) & 0xFF;
              ib_fcode = (sa_data[1] >> 0) & 0xFF;
              ib_addr = sa_data[2];
              ib_len = sa_data[3] & 0x0F;
              if (ib_run_transaction(cm_ptr->ib_ptr, ib_id, ib_fcode, ib_addr, ib_len, (uint16_t*)&sa_data[4]) > 0){
                ib_get_answer_data(cm_ptr->ib_ptr, (uint8_t*)&sa_data[4], ib_len*2);
              }
              mko_rt_write_to_subaddr(cm_ptr->mko_rt_ptr, CM_MKO_SA_TECH_CMD, (uint16_t*)sa_data);
              break;
            case (TCMD_SET_MKO_BC):
              sa_data[0] |= 0x0100;
              //
              memcpy((uint8_t*)data_arr, (uint8_t*)&sa_data[4], 28*2);
              mko_bc_set_bus(&mko_bc, sa_data[2]);
              mko_bc_transaction_start_by_cw(cm_ptr->mko_bc_ptr, sa_data[1], data_arr);
              mko_rt_write_to_subaddr(cm_ptr->mko_rt_ptr, CM_MKO_SA_TECH_RD, data_arr);
              //
              mko_rt_write_to_subaddr(cm_ptr->mko_rt_ptr, CM_MKO_SA_TECH_CMD, (uint16_t*)sa_data);
              break;
            case (TCMD_WD_MCU_RESET):
              sa_data[0] |= 0x0100;
              //
              if (sa_data[1] == 0xA55A){
                printf("Rst by WDG ");
                Timer_Delay(1, 30000);
                printf("Error\n");
                NVIC_SystemReset();
              }
              //
              mko_rt_write_to_subaddr(cm_ptr->mko_rt_ptr, CM_MKO_SA_TECH_CMD, (uint16_t*)sa_data);
              break;
            case (TCMD_GO_TO_TEST_MODE):
              sa_data[0] |= 0x0100;
              //
              if (sa_data[1] == 0xDEAD){
                printf("Go to test mode ");
                // сброс ожидания запуска интервалов
                cm_reset_start_waiting(&cm);
                // установка интервала измерения и системного интервала
                cm_set_interval_value(&cm, 0, 5);
                cm_set_interval_value(&cm, 1, 10);
                // сброс ожидания включения питания
                pwr_reset_init_delay(&pwr);
                // установка включения модулей с паузой 500 мс
                uint8_t pwr_state[PWR_CH_NUMBER] = PWR_DEFAULT_STATE;
                uint8_t pwr_type[PWR_CH_NUMBER] = PWR_CHANNELS_TYPES;
                for (uint8_t ch_num = 0; ch_num < PWR_CH_NUMBER; ch_num++){
                  if (pwr_type[ch_num] != PWR_CH_CTRL_NU) pwr_queue_put_cmd(&pwr, 500, ch_num, pwr_state[ch_num], 0);
                }
              }
              //
              mko_rt_write_to_subaddr(cm_ptr->mko_rt_ptr, CM_MKO_SA_TECH_CMD, (uint16_t*)sa_data);
              break;
            default:
              break;
          }
          break;
        default:
        //
        break;
        }
        break;
    case 2:
        // чтение из ПА для команд
    case 3:
      break;
  }
}

/**
 * @brief обработчик отладочных команд через ВШ
 * 
 * @param cm_ptr указатель на объект управления ЦМ
 */
void cm_dbg_ib_command_handler(typeCMModel* cm_ptr)
{
	typeFrameStruct frame;
	//
	if (cm_ptr->ib_ptr->command_frame_flag){
		cm_ptr->ib_ptr->command_frame_flag = 0;
		if (cm_ptr->ib_ptr->command_frame.dev_id == CM_SELF_MB_ID){
			if (cm_ptr->ib_ptr->command_frame.f_code == MB_F_CODE_16){
				printf("CM DBG CMD: ID <%d>, FC <%d>, RA <%d> \n", cm_ptr->ib_ptr->command_frame.dev_id, cm_ptr->ib_ptr->command_frame.f_code, cm_ptr->ib_ptr->command_frame.reg_addr);
				switch(cm_ptr->ib_ptr->command_frame.reg_addr){
					case CM_DBG_CMD_SWITCH_ON_OFF:
						cm_ptr->ib_ptr->global_dbg_flag = __REV16(cm_ptr->ib_ptr->command_frame.data[0]) & 0x01;
						break;
					case CM_DBG_CMD_CM_RESET:
						printf("Rst by WDG ");
						Timer_Delay(1, 100000);
						printf("Error\n");
						break;
					case CM_DBG_CMD_CM_CHECK_MEM:
						fr_mem_check(&cm_ptr->mem);
						break;
					case CM_DBG_CMD_CM_INIT:
						if (__REV16(cm_ptr->ib_ptr->command_frame.data[0]) == 0xAA55) __main_base_init();
						break;
					case CM_DBG_CMD_ARCH_REQUEST:
						if (__REV16(cm_ptr->ib_ptr->command_frame.data[0]) == 0x0000){
							fr_mem_read_data_frame(&cm_ptr->mem, (uint8_t*)&frame);
							mko_rt_write_to_subaddr(cm_ptr->mko_rt_ptr, CM_MKO_SA_ARCH_READ_CM, (uint16_t*)&frame);
						}
						break;
				}
			}
		}
	}
}

void ADC_SEQ0_CallBack(void)
{
  volatile int i;
  ADC->IC = 1;  //interrupt flag reset
  for(i=0; i<ADC_CHANNEL_NUM; i++)  adc.data[i] = ADC->SEQ[0].SFIFO; // (первое чтение - пустое)
}

void SysTick_Handler(void) 
{
  tp_timer_handler(&tp, 1000);
}

void INT_PORT_CallBack(GPIO_TypeDef *port, uint8_t line)
{
  uint8_t irq_source_num = 0;
  irq_source_num = oai_cm_io_find(&gpio, port, line);
  // Важно, в данном блок происходит связь команды управления и выходов (пример: OUT_CMD_1 <-> KU_CMD_WATCH_MODE_ON)
  switch(irq_source_num){
    default:
      // обработка прерывания от GPIO
    break;
  }
}

void MILSTD0_IRQHandler(void)
{
  mko_rt_irq_rx_callback(&mko_rt);
}

void HardFault_Handler(void)
{
  __DSB();
  while(1){
  }
}

void MemManage_Handler(void)
{
  __DSB();
  while(1){
  }
}
void BusFault_Handler(void)
{
  __DSB();
  while(1){
  }
}

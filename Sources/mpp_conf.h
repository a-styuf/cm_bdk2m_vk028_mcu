#ifndef _MPP_CONF_H_
#define _MPP_CONF_H_

// Настройки каналов МПП
#define MPP_DEV_NUM (8)
// !!настройки уставки МПП (offset) количество должно совпадать с MPP_DEV_NUM!!
#define MPP_DEF_OFFSET {0xF02, 0xF03, 0xF04, 0xF05, 0xF06, 0xF07, 0xF08, 0xF09}
// адреса МПП на внутренней шине
#define MPP_ID {4, 5, 6, 7, 8, 9, 3, 3}
// номер канала, используемый устройством МПП
#define MPP_CHANNENUM_ID {0, 0, 0, 0, 0, 0, 0, 1}
// используется ли канал в наборе помеховой матрицы
#define MPP_MATRIX_ENA {0, 0, 0, 0, 0, 0, 0, 0}

#define MPP_CHAN_K {+671.5E-6, +149.1E-4, +157.3E-4, +156.2E-4, 1.0E0, 1.0E0, 1.0E0, 1.0E0}
#define MPP_CHAN_B {-139.2E-2, -704.9E-2, -710.8E-2, -733.2E-2, 0.0E0, 0.0E0, 0.0E0, 0.0E0}

#endif

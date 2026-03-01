#ifndef TIMER_MEASURE_H
#define TIMER_MEASURE_H

#include "main.h"
#include <stdint.h>

/* Időmérő struktúra - DWT ciklusokat használ */
typedef struct {
    uint32_t start_cycles;
    uint32_t end_cycles;
    uint32_t elapsed_cycles;
    float elapsed_us;
    float elapsed_ms;
} TimerMeasure_t;

/**
 * @brief DWT timer inicializálása (csak egyszer kell meghívni!)
 */
void Timer_Init(void);

/**
 * @brief Időmérés indítása
 */
void Timer_Start(TimerMeasure_t *timer);

/**
 * @brief Időmérés befejezése és kiszámítása (us-ben és ms-ben)
 */
void Timer_Stop(TimerMeasure_t *timer);

/**
 * @brief Eltelt idő lekérdezése (us - mikroszekundum)
 */
float Timer_GetElapsed_us(TimerMeasure_t *timer);

/**
 * @brief Eltelt idő lekérdezése (ms)
 */
float Timer_GetElapsed_ms(TimerMeasure_t *timer);

/**
 * @brief Debug: Eltelt idő kiiratása a konzolra
 */
void Timer_PrintElapsed(const char *label, TimerMeasure_t *timer);

#endif /* TIMER_MEASURE_H */


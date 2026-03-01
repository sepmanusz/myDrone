#include "timer_measure.h"
#include "uart_printf.h"
#include "core_cm7.h"

extern void uart_printf(const char *format, ...);

/* CPU frekvencia (216 MHz az STM32F722-hez) */
#define CPU_FREQUENCY_HZ 216000000U
#define CYCLES_PER_MICROSECOND (CPU_FREQUENCY_HZ / 1000000U)  /* 216 ciklus = 1 us */

/**
 * @brief DWT timer inicializálása
 * Az CYCCNT regisztert engedélyezi a méréshez
 */
void Timer_Init(void)
{
    /* DWT aktiválása (Debug Exception and Monitor Control Register) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    
    /* CYCCNT aktiválása (Cycle Counter) */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    /* Számláló nullázása */
    DWT->CYCCNT = 0;
    
    uart_printf("Timer inicializalva (DWT, us felbontas)\r\n");
}

/**
 * @brief Időmérés indítása - CPU ciklusokat tárrol
 */
void Timer_Start(TimerMeasure_t *timer)
{
    if (timer == NULL) return;
    timer->start_cycles = DWT->CYCCNT;
}

/**
 * @brief Időmérés befejezése - kiszámítja az eltelt időt
 */
void Timer_Stop(TimerMeasure_t *timer)
{
    if (timer == NULL) return;
    timer->end_cycles = DWT->CYCCNT;
    
    /* Eltelt ciklusok számítása (overflow kezelése) */
    if (timer->end_cycles >= timer->start_cycles) {
        timer->elapsed_cycles = timer->end_cycles - timer->start_cycles;
    } else {
        /* CYCCNT overflow esetén (32-bit számláló) */
        timer->elapsed_cycles = (0xFFFFFFFF - timer->start_cycles) + timer->end_cycles;
    }
    
    /* Konvertálás mikroszekund és milliszekundba */
    timer->elapsed_us = (float)timer->elapsed_cycles / (float)CYCLES_PER_MICROSECOND;
    timer->elapsed_ms = timer->elapsed_us / 1000.0f;
}

/**
 * @brief Eltelt idő lekérdezése us-ben
 */
float Timer_GetElapsed_us(TimerMeasure_t *timer)
{
    if (timer == NULL) return 0.0f;
    return timer->elapsed_us;
}

/**
 * @brief Eltelt idő lekérdezése ms-ben
 */
float Timer_GetElapsed_ms(TimerMeasure_t *timer)
{
    if (timer == NULL) return 0.0f;
    return timer->elapsed_ms;
}

/**
 * @brief Debug: Eltelt idő kiiratása - integer alapú formázás
 */
void Timer_PrintElapsed(const char *label, TimerMeasure_t *timer)
{
    if (timer == NULL || label == NULL) return;
    
    if (timer->elapsed_us < 1000.0f) {
        /* Mikroszekundum pontossággal */
        uint32_t us_int = (uint32_t)timer->elapsed_us;
        uint32_t us_frac = (uint32_t)((timer->elapsed_us - us_int) * 100);
        uart_printf("[%s] Eltelt ido: %lu.%02lu us\r\n", label, us_int, us_frac);
    } else {
        /* Milliszekund pontossággal */
        uint32_t ms_int = (uint32_t)timer->elapsed_ms;
        uint32_t ms_frac = (uint32_t)((timer->elapsed_ms - ms_int) * 100);
        uart_printf("[%s] Eltelt ido: %lu.%02lu ms\r\n", label, ms_int, ms_frac);
    }
}


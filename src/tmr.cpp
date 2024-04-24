
#include <Arduino.h>
#include <time.h>

//int Timeout = 1;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

volatile uint32_t system_timer;

void IRAM_ATTR timer_int()
{
  system_timer++;
}

void init_timeout_interrupt(void)
{
    timer = timerBegin(0, 80, true);                  //timer 0, div 80
    timerAttachInterrupt(timer, &timer_int, true);  //attach callback
    timerAlarmWrite(timer,  1000, true); //set time in us
    timerAlarmEnable(timer); //enable interrupt
}

/*---------------------------------------------------------------------------*/
/* uint32_t set_ttimeout(uint16_t ticks)                            */
/*---------------------------------------------------------------------------*/
uint32_t set_ttimeout(uint16_t ticks)
{
  return (system_timer + ticks);
}
/*---------------------------------------------------------------------------*/
/* uint32_t set_ttimeout(uint16_t ticks)                                     */
/*---------------------------------------------------------------------------*/
uint32_t set_ttimeout_sec(uint16_t ticks)
{
  return (system_timer + (uint32_t)ticks * 1000L);
}

/*---------------------------------------------------------------------------*/
/* uint32_t set_ttimeout(uint16_t ticks)                                     */
/*---------------------------------------------------------------------------*/
uint32_t set_ttimeout_sec_M(uint16_t ticks)
{
  return (system_timer + (uint32_t)ticks);
}

/*---------------------------------------------------------------------------*/
/* int chk_timeout(uint32_t timeout)                                         */
/*---------------------------------------------------------------------------*/
bool chk_timeout(uint32_t timeout)
{
  if ((long)(system_timer - timeout) >= 0)
    return (true);
  else
    return (false);
}
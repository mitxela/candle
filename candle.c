#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "hardware/structs/systick.h"

#define IR_SENSOR 28
#define MOTOR 29

// matrix on GPIO 0 to 15, 16, 26
#define ANODES   0x000000FF
#define CATHODES 0x0401FF00

// cols are anodes
// rows are cathodes (ten available)

#define ANGULAR_RESOLUTION 24

// one slice contains both halves, second half of buffer should be mirror of first

uint32_t framebuffer[ANGULAR_RESOLUTION][ 8 ] = {};
uint32_t period = 0;

#define SIG_START 1
#define MOTOR_TIMEOUT_MS 250

#define SYSTICK_RVR 0x00FFFFFF

void draw_slice(){

//#define scantime 100
  uint32_t scantime = period * 0.125*(1.0/24.0/(10.5+7.5+4.5+1.5+1.5+4.5+7.5+10.5)/(125));

  gpio_put_all( 0x00015500 |(1<<0) );
  sleep_us(10.5*scantime);

  gpio_put_all( 0x0400AA00 |(1<<1) );
  sleep_us(7.5*scantime);

  gpio_put_all( 0x00015500 |(1<<2) );
  sleep_us(4.5*scantime);

  gpio_put_all( 0x0400AA00 |(1<<3) );
  sleep_us(1.5*scantime);

  gpio_put_all( 0x00015500 |(1<<4) );
  sleep_us(1.5*scantime);

  gpio_put_all( 0x0400AA00 |(1<<5) );
  sleep_us(4.5*scantime);

  gpio_put_all( 0x00015500 |(1<<6) );
  sleep_us(7.5*scantime);

  gpio_put_all( 0x0400AA00 |(1<<7) );
  sleep_us(10.5*scantime);


  gpio_put_all( 0 );

}

void draw_slice_temp(){

//#define scantime 100
  uint32_t scantime = period * 0.125*(1.0/24.0/(10.5+7.5+4.5+1.5+1.5+4.5+7.5+10.5)/(125));

  gpio_put_all( 0x0400AA00 |(1<<0) );
  sleep_us(10.5*scantime);

  gpio_put_all( 0x00015500 |(1<<1) );
  sleep_us(7.5*scantime);

  gpio_put_all( 0x0400AA00 |(1<<2) );
  sleep_us(4.5*scantime);

  gpio_put_all( 0x00015500 |(1<<3) );
  sleep_us(1.5*scantime);

  gpio_put_all( 0x0400AA00 |(1<<4) );
  sleep_us(1.5*scantime);

  gpio_put_all( 0x00015500 |(1<<5) );
  sleep_us(4.5*scantime);

  gpio_put_all( 0x0400AA00 |(1<<6) );
  sleep_us(7.5*scantime);

  gpio_put_all( 0x00015500 |(1<<7) );
  sleep_us(10.5*scantime);


  gpio_put_all( 0 );

}

void core1_entry(void){

  while (1) {
  start:

    while (multicore_fifo_pop_blocking() != SIG_START);

    for (int i = 0; i< ANGULAR_RESOLUTION; i++) {
      if (i&1) draw_slice(); else draw_slice_temp();

      if ( multicore_fifo_get_status() &(1<<0) ) goto start;
    }

    // delay but break early on signal
    for (int i = 0; i< MOTOR_TIMEOUT_MS*1000; i++) {
      if ( multicore_fifo_get_status() &(1<<0) ) goto start;
      sleep_us(1);
    }

    // motor off
    pwm_set_gpio_level(MOTOR, 0);
  }
}


int main(){

  gpio_init_mask(ANODES|CATHODES|(1<<MOTOR)|(1<<IR_SENSOR));

  gpio_set_dir(IR_SENSOR, 0);
  gpio_set_dir_out_masked(ANODES|CATHODES);

  gpio_set_function(MOTOR, GPIO_FUNC_PWM); // channel 6B

  gpio_set_function(MOTOR, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(MOTOR);
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, 4.f);
  pwm_init(slice_num, &config, true);


  multicore_launch_core1(core1_entry);

  systick_hw->csr = M0PLUS_SYST_CSR_ENABLE_BITS | M0PLUS_SYST_CSR_CLKSOURCE_BITS;
  systick_hw->rvr = SYSTICK_RVR;

  uint32_t timing = 0;

  while (1) {


    while (gpio_get(IR_SENSOR) == 1);
    uint32_t t0 = systick_hw->cvr;
    multicore_fifo_push_blocking(SIG_START);

    // Target 1800RPM or 30RPS, period = 4166666 cycles @ 125MHz
    // 1200RPM or 20rps - > 6250000

    period = SYSTICK_RVR - t0;
    if (period > 6250000) pwm_set_gpio_level(MOTOR, 0.9*65535);
    else pwm_set_gpio_level(MOTOR, 0.6*65535);

    while (gpio_get(IR_SENSOR) == 0) sleep_us(1);

    


  }
}


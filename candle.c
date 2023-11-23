#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define IR_SENSOR 28
#define MOTOR 29

// matrix on GPIO 0 to 15, 16, 26
#define ANODES   0x000000FF
#define CATHODES 0x0401FF00

// cols are anodes
// rows are cathodes (ten available)

void draw_slice(){

#define scantime 1

  gpio_put_all( 0x00015500 |(1<<0) );
  sleep_ms(scantime);

  gpio_put_all( 0x0400AA00 |(1<<1) );
  sleep_ms(scantime);

  gpio_put_all( 0x00015500 |(1<<2) );
  sleep_ms(scantime);

  gpio_put_all( 0x0400AA00 |(1<<3) );
  sleep_ms(scantime);

  gpio_put_all( 0x00015500 |(1<<4) );
  sleep_ms(scantime);

  gpio_put_all( 0x0400AA00 |(1<<5) );
  sleep_ms(scantime);

  gpio_put_all( 0x00015500 |(1<<6) );
  sleep_ms(scantime);

  gpio_put_all( 0x0400AA00 |(1<<7) );
  sleep_ms(scantime);


  gpio_put_all( 0 );

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


  while(1) draw_slice();


  while (1) {

    sleep_us(1);

    while (gpio_get(IR_SENSOR) == 1) sleep_ms(1);

    pwm_set_gpio_level(MOTOR, 0.8*65535);

    while (gpio_get(IR_SENSOR) == 0) sleep_ms(1);

    pwm_set_gpio_level(MOTOR, 0);


  }
}


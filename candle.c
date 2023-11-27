#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
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

volatile bool active_frame = 0;
uint32_t framebuffer[2][ANGULAR_RESOLUTION][ 8 ];
volatile uint32_t period = 0;
volatile bool idle = 1;
int mode = 0;

#include "fdata-cube-rotate.h"
#include "fdata-liquid.h"
#include "fdata-fire.h"
#include "font.h"

#define SIG_START 1
#define MOTOR_TIMEOUT_MS 250
#define IDLE_TIMEOUT_MS 2000

#define SYSTICK_RVR 0x00FFFFFF

// Max 0x00FFFFFF cycles = ~134ms
void sleep_cycles_break(uint32_t cycles){
  systick_hw->cvr = systick_hw->rvr;
  uint32_t until = systick_hw->rvr - cycles;
  while (systick_hw->cvr > until) {
    if ( multicore_fifo_get_status() &(1<<0) ) return;
  }
}

void draw_slice( uint32_t slice ){

#define scantime (1.0/24.0/(10.5+7.5+4.5+1.5+1.5+4.5+7.5+10.5))

  gpio_put_all( framebuffer[active_frame][slice][0] );
  sleep_cycles_break(10.5*scantime*period);

  gpio_put_all( framebuffer[active_frame][slice][1] );
  sleep_cycles_break(7.5*scantime*period);

  gpio_put_all( framebuffer[active_frame][slice][2] );
  sleep_cycles_break(4.5*scantime*period);

  gpio_put_all( framebuffer[active_frame][slice][3] );
  sleep_cycles_break(1.5*scantime*period);

  gpio_put_all( framebuffer[active_frame][slice][4] );
  sleep_cycles_break(1.5*scantime*period);

  gpio_put_all( framebuffer[active_frame][slice][5] );
  sleep_cycles_break(4.5*scantime*period);

  gpio_put_all( framebuffer[active_frame][slice][6] );
  sleep_cycles_break(7.5*scantime*period);

  gpio_put_all( framebuffer[active_frame][slice][7] );
  sleep_cycles_break(10.5*scantime*period);


  gpio_put_all( 0 );

}

void core1_entry(void){

  systick_hw->csr = M0PLUS_SYST_CSR_ENABLE_BITS | M0PLUS_SYST_CSR_CLKSOURCE_BITS;
  systick_hw->rvr = SYSTICK_RVR;

  while (1) {
  start:

    while (multicore_fifo_pop_blocking() != SIG_START);

    for (int i = 0; i< ANGULAR_RESOLUTION; i++) {
      //if (i&1) draw_slice(0); else draw_slice(1);
      draw_slice(i);

      if ( multicore_fifo_get_status() &(1<<0) ) goto start;
    }

    // delay but break early on signal
    for (int i = 0; i< MOTOR_TIMEOUT_MS; i++) {
      if ( multicore_fifo_get_status() &(1<<0) ) goto start;
      sleep_cycles_break(125000);
    }

    // motor off
    pwm_set_gpio_level(MOTOR, 0);

    for (int i = 0; i< IDLE_TIMEOUT_MS; i++) {
      if ( multicore_fifo_get_status() &(1<<0) ) goto start;
      sleep_cycles_break(125000);
    }

    idle = 1;

  }
}

static inline void check_battery(){
  // LDO is RT9193-33, dropout 220mV at 300mA
  // adc ref is 3.3 when vbat >=3.52
  // at 4.2, adc reads 4096*(4.2/2)/3.3 = 2606.55
  // at 3.7, adc reads 4096*(3.7/2)/3.3 = 2296.24
  // at 3.52, adc reads 4096*(3.52/2)/3.3 = 2184.53
  // at 3.3, adc reads 4096*(3.3/2)/(3.3-0.22) = 2194.29
  // at 3.0, adc reads 4096*(3.0/2)/(3.0-0.22) = 2210.07

  // Essentially when it gets below 2210, it could be any voltage from 3 to 3.55

//measured: 2120 was about 3.57V
//Even at >3.8v, reading dipped below 2120 while motor running

  uint16_t raw = adc_read();
  if (raw > 2120) return; //2210

  pwm_set_gpio_level(MOTOR, 0);
  // show warning message and hang
  while(1) {
    gpio_put_all( (CATHODES&0xAA555555) |(1<<3) );
    sleep_ms(1);
    gpio_put_all( (CATHODES&0x55AAAAAA) |(1<<4) );
    sleep_ms(1);
    gpio_put_all(0);
    sleep_ms(80);
  }

}

void load_frame(const uint32_t* data){
  int k=0;
  bool n = !active_frame;
  for (int i =0;i<ANGULAR_RESOLUTION; i++) {
    for (int j =0;j<8;j++) {
      framebuffer[n][i][j] = data[k++];
    }
  }

  active_frame = !active_frame;
}

void clr(){
  for (int i =0;i<ANGULAR_RESOLUTION; i++) {
    for (int j =0;j<8;j++) framebuffer[active_frame][i][j] = CATHODES|(1<<j);
  }
}

// r: [0...7]
void set_voxel(uint32_t r, uint32_t theta, uint32_t z) {
  uint32_t rownum;
  //0x0401FF00
  if (z == 8) rownum == 16;
  else if (z == 9) rownum == 26;
  else rownum = z+8;

  framebuffer[active_frame][theta][r] &= ~(1<<rownum);
  framebuffer[active_frame][(theta+12)%24][7-r] &= ~(1<<rownum);

}

// r: [0...7]
void set_voxel_reflect(uint32_t r, uint32_t theta, uint32_t z) {
  uint32_t rownum;
  //0x0401FF00
  if (z == 8) rownum == 16;
  else if (z == 9) rownum == 26;
  else rownum = z+8;

  if (z>9 || theta>23 || r>7) return;

  framebuffer[active_frame][theta][r] &= ~(1<<rownum);
  framebuffer[active_frame][(theta+12)%24][r] &= ~(1<<rownum);

}

void load_character(uint32_t r, uint32_t theta, char c){
  if (c>'~' || c<'!') return;
  const uint8_t * a = &font[ 5*(c-'!')];
  for (int i=0;i<5;i++) {
    for (int j=0;j<8;j++) {
      if ((a[i] & (1<<j)) ==0) set_voxel_reflect(r+5-i, theta, 1+j);
    }
  }
}

void load_text(){
  static const char msg[] = "This is a test, a very long message";
  static int f = -7, c = 0, slow=0;

  if (++slow<3) return;
  slow=0;

  f++;
  if (f>7) {
    f-=6;
    c++;
    if (c == (sizeof msg) -2) c=0;
  }
  clr();

  load_character(f,0,msg[c]);
  load_character(f-6,0,msg[c+1]);
  load_character(f-12,0,msg[c+2]);

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

  adc_init();
  adc_gpio_init(27);
  adc_select_input(1);

  multicore_launch_core1(core1_entry);

  systick_hw->csr = M0PLUS_SYST_CSR_ENABLE_BITS | M0PLUS_SYST_CSR_CLKSOURCE_BITS;
  systick_hw->rvr = SYSTICK_RVR;

  uint32_t f = 0;
  load_frame(&framedata_liquid[f][0][0]);

  uint32_t timing = 0;

  while (1) {

    while (gpio_get(IR_SENSOR) == 1);
    uint32_t t0 = systick_hw->cvr;
    multicore_fifo_push_blocking(SIG_START);
    systick_hw->cvr = SYSTICK_RVR;
    period = SYSTICK_RVR - t0;

    if (idle) {
      check_battery();
      f=0;
      //mode++; if (mode>4) mode=0;
    }
    idle = 0;

    // Target 1800RPM or 30RPS, period = 4166666 cycles @ 125MHz
    // 1200RPM or 20rps -> 6250000
    // 24rps = 5208333 cycles

    if (period > 6250000) pwm_set_gpio_level(MOTOR, 0.9*65535);
    else pwm_set_gpio_level(MOTOR, 0.6*65535);

    #define load_static( data ) \
      if (++f>= sizeof data / sizeof data[0]) f=0; \
      load_frame(&data[f][0][0]);

    if (mode == 0) {
 load_text();
      //load_frame(&framedata_cube[0][0][0]);
    }
    else if (mode == 1) {
      load_static(framedata_cube)
    }
    else if (mode == 2) {
      load_static(framedata_liquid)
    }
    else if (mode == 3){
      load_static(framedata_fire)
    }
    else if (mode == 4) {
      load_text();
    }

    while (gpio_get(IR_SENSOR) == 0) sleep_us(1);


  }
}


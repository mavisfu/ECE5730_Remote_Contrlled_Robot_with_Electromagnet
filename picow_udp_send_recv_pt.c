/**
 * Wenyi Fu (wf223@cornell.edu)
 * Klora Wang (hw768@cornell.edu)
 * 
 * Romote-controlled Robot with Electromagnet Implemented on RP2040
 * 
 * Dec 7, 2023
 */
#include <string.h>
#include <stdlib.h>
#include <pico/multicore.h>
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/uart.h"


#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/opt.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/dns.h"
#include "lwip/netif.h"

#include <stdio.h>
#include <math.h>



#include "hardware/pwm.h"
#include "hardware/irq.h"

#include "pt_cornell_rp2040_v1_1_2.h"

// PWM wrap value and clock divide value
// For a CPU rate of 125 MHz, this gives
// a PWM frequency of 1 kHz.
#define WRAPVAL 5000
#define CLKDIV 25.0f

// GPIO control left motor, PWM GPIO 10
#define MOTOR1_SELECT1 3
#define MOTOR1_SELECT2 4
#define LEFT_MOTOR 10
// GPIO control right motor, PWM GPIO 11
#define MOTOR2_SELECT1 7
#define MOTOR2_SELECT2 8
#define RIGHT_MOTOR 11

// Electro control 
#define ELECTRO_CONTROL 17
#define ELECTRO_LED 18

// LED indicate connection to wifi
#define WIFI_LED 2

// Variable to hold PWM slice number
uint slice_num ;
uint slice_num1 ;

// PWM duty cycle
volatile int control ;
volatile int old_control ;

volatile int control1 ;

volatile int STATE = 0;
volatile int LAST_STATE = 0;

// wifi setup - hotspt/wifi name & password
char ssid[] = "********"; 
char pass[] = "********"; 

#define UDP_PORT 4444

#define BEACON_MSG_LEN_MAX 127
#define BEACON_TARGET "172.20.10.5"
#define BEACON_INTERVAL_MS 1000

// ======================================
// udp constants
// ======================================
#define UDP_MSG_LEN_MAX 32
#define UDP_TARGET "172.20.10.5" // my laptop addresss
#define UDP_INTERVAL_MS 10


char recv_data[UDP_MSG_LEN_MAX];

// payload to led blink
int blink_time ;
char action ;
// interthread communicaitqoin
struct pt_sem new_udp_recv_s, new_udp_send_s ;

static struct udp_pcb *udpecho_raw_pcb;
struct pbuf *p ;

static void
udpecho_raw_recv(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                 const ip_addr_t *addr, u16_t port)
{
  LWIP_UNUSED_ARG(arg);

  if (p != NULL) {
    //printf("p payload in call back: = %s\n", p->payload);
    memcpy(recv_data, p->payload, UDP_MSG_LEN_MAX);
    // can signal from an ISR -- BUT NEVER wait in an ISR
    PT_SEM_SIGNAL(pt, &new_udp_recv_s) ;
    
    /* free the pbuf */
    pbuf_free(p);
  }
  else printf("NULL pt in callback");
}


// ===================================
// Define the recv callback 
void 
udpecho_raw_init(void)
{
  udpecho_raw_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
  p = pbuf_alloc(PBUF_TRANSPORT, UDP_MSG_LEN_MAX+1, PBUF_RAM);

  if (udpecho_raw_pcb != NULL) {
    err_t err;
    // netif_ip4_addr returns the picow ip address
    err = udp_bind(udpecho_raw_pcb, netif_ip4_addr(netif_list), UDP_PORT); //DHCP addr

    if (err == ERR_OK) {
      udp_recv(udpecho_raw_pcb, udpecho_raw_recv, NULL);
      //printf("Set up recv callback\n");
    } else {
      printf("bind error");
    }
  } else {
    printf("udpecho_raw_pcb error");
  }
}

// only for testing
void run_udp_beacon() {
    struct udp_pcb* pcb = udp_new();

    ip_addr_t addr;
    ipaddr_aton(BEACON_TARGET, &addr);

    int counter = 0;
    while (true) {
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, BEACON_MSG_LEN_MAX+1, PBUF_RAM);
        char *req = (char *)p->payload;
        memset(req, 0, BEACON_MSG_LEN_MAX+1);
        snprintf(req, BEACON_MSG_LEN_MAX, "%d\n", counter);
        err_t er = udp_sendto(pcb, p, &addr, UDP_PORT);
        pbuf_free(p);
        if (er != ERR_OK) {
            printf("Failed to send UDP packet! error=%d", er);
        } else {
            printf("Sent packet %d\n", counter);
            counter++;
        }

        // Note in practice for this simple UDP transmitter,
        // the end result for both background and poll is the same

#if PICO_CYW43_ARCH_POLL
        // if you are using pico_cyw43_arch_poll, then you must poll periodically from your
        // main loop (not from a timer) to check for Wi-Fi driver or lwIP work that needs to be done.
        cyw43_arch_poll();
        sleep_ms(BEACON_INTERVAL_MS);
#else
        // if you are not using pico_cyw43_arch_poll, then WiFI driver and lwIP work
        // is done via interrupt in the background. This sleep is just an example of some (blocking)
        // work you might be doing.
        sleep_ms(BEACON_INTERVAL_MS);
#endif
    }
}

// ==================================================
// udp recv thread
// ==================================================
static PT_THREAD (protothread_udp_recv(struct pt *pt))
{
    PT_BEGIN(pt);
    
     // data structure for interval timer
     PT_INTERVAL_INIT() ;

      while(1) {
        // wait for new packet
        // MUST be an integer format number!!
        PT_SEM_WAIT(pt, &new_udp_recv_s) ;

        // process packet and signal udp send thread
        sscanf(recv_data, "%c", &action) ; 
        // sscanf(recv_data, "%d", &blink_time) ;  

        // for blinky testing
        // if (blink_time == 100) {
        //     STATE = 1;
        // }

        // Keyboard control characters
        if (action == 'w') {
            LAST_STATE = STATE;
            STATE = 1;
        }
        if (action == 'a') {
            LAST_STATE = STATE;
            if (LAST_STATE == 1 || LAST_STATE == 4) {
                STATE = 3;
            }
            else if (LAST_STATE == 2 || LAST_STATE == 6) {
                STATE = 5;

            }
        }
        if (action == 'd') {
            LAST_STATE = STATE;
            if (LAST_STATE == 1 || LAST_STATE == 3) {
                STATE = 4;
            }
            else if (LAST_STATE == 2 || LAST_STATE == 5) {
                STATE = 6;

            }
        }
        if (action == 's') {
            LAST_STATE = STATE;
            STATE = 2;
        }
        if (action == 'c') {
            LAST_STATE = STATE;
            STATE = 0;
        }
        if (action == 'k') {
            gpio_put(ELECTRO_CONTROL, 1);
            gpio_put(ELECTRO_LED, 1);
        }
        if (action == 'l') {
            gpio_put(ELECTRO_CONTROL, 0);
            gpio_put(ELECTRO_LED, 0);
        }




       // tell send threead 
        PT_SEM_SIGNAL(pt, &new_udp_send_s) ;

        PT_YIELD_INTERVAL(10) ;
        //
        // NEVER exit while
      } // END WHILE(1)
    PT_END(pt);
} // blink thread

// ==================================================
// toggle cyw43 LED  
// this is really just a test of multitasking
// compatability with LWIP
// ==================================================
static PT_THREAD (protothread_toggle_cyw43(struct pt *pt))
{
    PT_BEGIN(pt);
    static bool LED_state = false ;
    //
     // data structure for interval timer
     PT_INTERVAL_INIT() ;
     // set some default blink time
     blink_time = 100 ;
     // echo the default time to udp connection
      PT_SEM_SIGNAL(pt, &new_udp_send_s) ;

      while(1) {
        //
        LED_state = !LED_state ;
        // the onboard LED is attached to the wifi module
        cyw43_arch_gpio_put(0, LED_state);
        // blink time is modifed by the udp recv thread
        PT_YIELD_INTERVAL(blink_time*1000) ;
        //
        // NEVER exit while
      } // END WHILE(1)
    PT_END(pt);
} // blink thread

// ======================================
// Motor control function 
// ======================================
void Stop() {
    gpio_put(MOTOR1_SELECT1, 0);
    gpio_put(MOTOR1_SELECT2, 0);
    gpio_put(MOTOR2_SELECT1, 0);
    gpio_put(MOTOR2_SELECT2, 0);
}
void Forward(int speed) {
    gpio_put(MOTOR1_SELECT1, 0);
    gpio_put(MOTOR1_SELECT2, 1);
    gpio_put(MOTOR2_SELECT1, 0);
    gpio_put(MOTOR2_SELECT2, 1);
    control = speed;
    control1 = speed;
}
void Reverse(int speed) {
    gpio_put(MOTOR1_SELECT1, 1);
    gpio_put(MOTOR1_SELECT2, 0);
    gpio_put(MOTOR2_SELECT1, 1);
    gpio_put(MOTOR2_SELECT2, 0);
    control = speed;
    control1 = speed;
}

void TurnLeft() {
    gpio_put(MOTOR1_SELECT1, 0);
    gpio_put(MOTOR1_SELECT2, 1);
    gpio_put(MOTOR2_SELECT1, 0);
    gpio_put(MOTOR2_SELECT2, 1);
    control = 3500;
    control1 = 1500;
}

void TurnRight() {
    gpio_put(MOTOR1_SELECT1, 0);
    gpio_put(MOTOR1_SELECT2, 1);
    gpio_put(MOTOR2_SELECT1, 0);
    gpio_put(MOTOR2_SELECT2, 1);
    control = 1500;
    control1 = 3500;
}

void ReverseTurnLeft() {
    gpio_put(MOTOR1_SELECT1, 1);
    gpio_put(MOTOR1_SELECT2, 0);
    gpio_put(MOTOR2_SELECT1, 1);
    gpio_put(MOTOR2_SELECT2, 0);
    control = 3500;
    control1 = 1500;
}

void ReverseTurnRight() {
    gpio_put(MOTOR1_SELECT1, 1);
    gpio_put(MOTOR1_SELECT2, 0);
    gpio_put(MOTOR2_SELECT1, 1);
    gpio_put(MOTOR2_SELECT2, 0);
    control = 1500;
    control1 = 3500;
}

// ======================================
// PWM interrupt service routine
// ======================================
void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(LEFT_MOTOR));

    // Switch state according to receiving data
    if (STATE == 0){
        Stop();
    }
    
    if (STATE == 1) {
        Forward(3000);

    }
    else if (STATE == 2) {
        Reverse(3000);

    }
    else if (STATE == 3) {
        
        TurnLeft();
        

    }
    else if (STATE == 4) {
        
        TurnRight();
        

    }
    else if (STATE == 5) {
        
        ReverseTurnLeft();
        

    }
    else if (STATE == 6) {
        
        ReverseTurnRight();
        

    }

    // Update duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, control1);
    
}

// ================================================
// User input thread - for debugging and testing
// ================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char classifier ;
    static int test_in ;
    static float float_in ;
    while(1) {
        sprintf(pt_serial_out_buffer, "0 duty cycle, 1 forward motor, 2 reverse motor, 3 turn left, 4 turn right, 5 turn on electro, 6 turn off: ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%c", &classifier) ;
        if (classifier=='0') {
            sprintf(pt_serial_out_buffer, "input a duty cycle (0-5000): ");
            serial_write ;
            // spawn a thread to do the non-blocking serial read
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%d", &test_in) ;
            STATE = 0;
            if (test_in > 5000) continue ;
            else if (test_in < 0) continue ;
            else {
                control = test_in ;
                control1 = test_in ;
            }
        }
        if (classifier=='1') {
            sprintf(pt_serial_out_buffer, "forward");
            // serial_write ;
            // // spawn a thread to do the non-blocking serial read
            // serial_read ;
            // // convert input string to number
            // sscanf(pt_serial_in_buffer,"%d", &test_in) ;
            STATE = 1;
        }
        if (classifier=='2') {
            sprintf(pt_serial_out_buffer, "reverse");
            // serial_write ;
            // // spawn a thread to do the non-blocking serial read
            // serial_read ;
            // // convert input string to number
            // sscanf(pt_serial_in_buffer,"%d", &test_in) ;
            STATE = 2;
        }
        if (classifier=='3') {
            sprintf(pt_serial_out_buffer, "turn left");
            STATE = 3;
        }
        if (classifier=='4') {
            sprintf(pt_serial_out_buffer, "turn right");
            STATE = 4;
        }
        if (classifier=='5') {
            gpio_put(ELECTRO_CONTROL, 1);
        }
        if (classifier=='6') {
            gpio_put(ELECTRO_CONTROL, 0);
        }
    }
    PT_END(pt) ;
}

// ================================================
// Main program - configuration of wifi, GPIO...
// ================================================
int main() {

    // Initialize stdio
    stdio_init_all();

    // Initialize LED GPIO
    gpio_init(WIFI_LED);
    gpio_set_dir(WIFI_LED, GPIO_OUT);

    // Connect to wifi
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_UK)) {
    printf("failed to initialise\n");
    return 1;
    }
    printf("initialised\n");
    
    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(ssid, pass, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
    printf("failed to connect\n");
    gpio_put(WIFI_LED, 0);
    return 1;
    }
    printf("connected\n");
    gpio_put(WIFI_LED, 1);
    printf("Connected: picoW IP addr: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    //   run_udp_beacon(); // only for testing 

    //============================
    // UDP recenve ISR routines
    //============================
    udpecho_raw_init();

    PT_SEM_INIT(&new_udp_recv_s, 0) ;

    // Set GPIO 
    gpio_init(MOTOR1_SELECT1);
    gpio_init(MOTOR1_SELECT2);
    gpio_set_dir(MOTOR1_SELECT1, GPIO_OUT);
    gpio_set_dir(MOTOR1_SELECT2, GPIO_OUT);
    
    gpio_init(MOTOR2_SELECT1);
    gpio_init(MOTOR2_SELECT2);
    gpio_set_dir(MOTOR2_SELECT1, GPIO_OUT);
    gpio_set_dir(MOTOR2_SELECT2, GPIO_OUT);

    gpio_init(ELECTRO_CONTROL);
    gpio_set_dir(ELECTRO_CONTROL, GPIO_OUT);
    gpio_init(ELECTRO_LED);
    gpio_set_dir(ELECTRO_LED, GPIO_OUT);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO 5 that it is allocated to the PWM
    gpio_set_function(LEFT_MOTOR, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_MOTOR, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2)
    slice_num = pwm_gpio_to_slice_num(10);
    // slice_num1 = pwm_gpio_to_slice_num(RIGHT_MOTOR);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    // pwm_clear_irq(slice_num1);
    // pwm_set_irq_enabled(slice_num1, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;
    // pwm_set_wrap(slice_num1, WRAPVAL) ;
    // pwm_set_clkdiv(slice_num1, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 3125);

    pwm_set_chan_level(slice_num, PWM_CHAN_B, 3125);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

    // pwm_set_mask_enabled((1u << slice_num1));

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
   
    // Adding desired thread
    pt_add_thread(protothread_udp_recv);
    pt_add_thread(protothread_toggle_cyw43) ;

    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

    cyw43_arch_deinit();

}

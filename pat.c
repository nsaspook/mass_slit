
// PIC18F1320 Configuration Bit Settings

#include <p18f1320.h>

// CONFIG1H
#pragma config OSC = INTIO2      // Oscillator Selection bits (INT oscillator, port functions)
#pragma config FSCM = ON        // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal External Switchover mode enabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bit (Brown-out Reset enabled)
// BORV = No Setting

// CONFIG2H
#pragma config WDT = ON        // Watchdog Timer Enable bit
#pragma config WDTPS = 4096    // Watchdog Timer Postscale Select bits

// CONFIG3H
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled, RA5 input pin disabled)

// CONFIG4L
#pragma config STVR = ON        // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF       // Low-Voltage ICSP Enable bit (Low-Voltage ICSP disabled)

// CONFIG5L
#pragma config CP0 = ON        // Code Protection bit (Block 0 (00200-000FFFh) not code-protected)
#pragma config CP1 = ON        // Code Protection bit (Block 1 (001000-001FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (00200-000FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (001000-001FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (00200-000FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (001000-001FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)


/*
 * Driver for slow speed RMS monitor/speed signal generator
 * std input 57 pulses per min, output 850Hz
 * Runs on the PIC18F1320, with watchdog and software checks to be
 * sure we only generate the correct signal when the motor is running.
 *
 * Version
 * 0.1  config chip to translate input pulses to the correct RPM signal
 * 0.2 rs-232 debug RX DIP-pin7, TX DIP-pin8
 * 1.0 beta production test version.
 * 1.1 reduce 'at RPM' time during spinup
 *
 * HiRose din connector 1 sig out (white), 2 sig ret(black), 3 shield/gnd(green), 4 24vdc power in(red), 5 rs-232 rx, 6 rs-232 tx
 *
 * Sensor connector:
 * Cherry gs100502 sensor	: BRN/Vcc-Pin4, BLK/OUT-Pin1, BLU/GND-Pin2
 * Honeywell LCZ260		: RED/Vcc-pin4, WHT/OUT=pin1, BLK/COM0pin2
 *
 * RMS Assy connector: 1 SLIT-FOR, 2 SLIT-REC, 3 +24, 4 +24, 6 power 24v gnd, 10 24v power gnd
 * 11 TACH sig, 12 TACH ref/gnd
 *
 * 120VAC motor power plug: 1 Black: Hot, 2 N/A, 3 White: Neutral, 4 Green: Ground
 * motor connections        1 Red: Hot,   2 N/C, 3 Blue: Neutral,  4 GreenL Ground
 * motor run cap red/black with red wire to power plug
 *
 * Power for the controller box is tapped from pin 3 or 4 for power, pin 10 for ground and connected to pin 4 on the RMS
 * original Molex sensor connector. This 24vdc is converted inside the controller with a isolated
 * ground DC-DC converter to 5vdc for the controller and Honeywell Hall-effect sensor.
 * The pic microcontroller senses the pulses from the Hall-effect device, checks for the correct
 * timing of signals and controller operation and then outputs a signal to emulate the signal
 * produced from the original high-speed motor at ~850hz. If the signal is lost, out of timing spec or
 * the internal software checks fail the uC will reboot to see it that will recover normal operation
 * with a good RPM signal.
 */

#include <timers.h>
#include <stdlib.h>
#include <usart.h>
#include <stdio.h>
#include <EEP.h>
#include "pat.h"
#include "blinker.h"
#include <string.h>

void tm_handler(void);
int16_t sw_work(void);
void init_rmsmon(void);
uint8_t init_rms_params(void);

#pragma udata
volatile struct V_data V;
volatile union Obits2 LEDS;
int8_t str[12];
#pragma udata access ACCESSBANK
volatile uint16_t timer0_off = TIMEROFFSET;
near uint16_t blink_speed;

const far rom int8_t VER[] = "\r\nAP_V_MS_1.0\r\n";
const far rom int8_t build_date[] = __DATE__, build_time[] = __TIME__;
const far rom int8_t
spacer0[] = " ",
	spacer1[] = "\r\n",
	status0[] = "\r\n OK ",
	status1[] = "\r\n Booting RPM converter ",
	status2[] = "\r\n RPMC waiting for signal ",
	status3[] = "\r\n RPMC spinning normal ",
	status4[] = "\r\n RPMC spinning low ",
	boot0[] = "\r\n Boot RCON ",
	boot1[] = "\r\n Boot STKPTR ";

#pragma code tm_interrupt = 0x8

void tm_int(void)
{
	_asm goto tm_handler _endasm
}
#pragma code

#pragma interrupt tm_handler

void tm_handler(void) // timer/serial functions are handled here
{
	static uint8_t led_cache = 0xff;

	/* check for expected interrupts */
	V.valid = FALSE;

	if (INTCONbits.INT0IF) {
		V.valid = TRUE;
		INTCONbits.INT0IF = FALSE;
		V.spin_count++;
		V.sleep_ticks = 0;
		RPMLED = !RPMLED;
	}

	if (PIR1bits.RCIF) { // is data from RS-232 port
		V.valid = TRUE;
		V.rx_data = RCREG;
		if (RCSTAbits.OERR) {
			RCSTAbits.CREN = 0; // clear overrun
			RCSTAbits.CREN = 1; // re-enable
		}
		V.comm = TRUE;
	}

	if (PIR1bits.TMR1IF) { //      Timer1 int handler for RPM frequency generator
		V.valid = TRUE;
		PIR1bits.TMR1IF = FALSE; //      clear int flag
		if (V.slew_freq < V.sample_freq) V.slew_freq += 10;
		if (V.slew_freq > V.sample_freq) V.slew_freq--;
		WriteTimer1(V.slew_freq); // this can change for ramp up/down simulation
		if (V.spinning) {
			RPMOUT = !RPMOUT; // generate the high speed motor timing pulses
		} else {
			V.slew_freq = SAMPLEFREQ_R;
		}
		V.hi_rez++;
	}

	if (INTCONbits.TMR0IF) { //      check timer0 irq time timer
		V.valid = TRUE;
		INTCONbits.TMR0IF = FALSE; //      clear interrupt flag
		WriteTimer0(timer0_off);

		if (V.stop_tick != OFF) {
			if (V.stop_tick > MAX_TICK)
				V.stop_tick = MAX_TICK;
			V.stop_tick--;
		}

		if (!(++V.mod_count % 2)) {
			// check for spin motor movement
			if (V.spin_count >= RPM_COUNT) {
				V.spinning = TRUE;
				V.stop_tick = MAX_TICK;
				if (V.motor_ramp < START_RAMP) {
					V.motor_ramp++;
					V.sample_freq = SAMPLEFREQ_S; // slower spin-up RPM signal to RDAC
					WriteTimer1(V.sample_freq);
					V.comm_state = 4;
				} else {
					V.sample_freq = SAMPLEFREQ; // normal RPM signal to RDAC
					V.comm_state = 3;
					if ((V.spin_count > V.max_freq) && (V.spin_count < SPIN_LIMIT_H)) V.max_freq = V.spin_count;
					V.hi_rez_count = V.hi_rez;
					V.hi_rez = 0;
				}
			} else {
				if (V.stop_tick == OFF) {
					V.spinning = FALSE;
					V.motor_ramp = OFF;
					RPMOUT = 0;
					V.sample_freq = SAMPLEFREQ_S; // slowdown RPM signal to RDAC
					V.comm_state = 2;
					V.sleep_ticks++;
				}
				if (V.stop_tick >= STOP_RAMP) {
					V.sample_freq = SAMPLEFREQ_S;
					WriteTimer1(V.sample_freq);
				}
			}
			V.spin_count = 0;
		}

		/* Start Led Blink Code */
		if (V.blink_alt) {
			if (V.blink & 0b00000001) LEDS.out_bits.b0 = !LEDS.out_bits.b0;
			if (V.blink & 0b00000010) LEDS.out_bits.b1 = !LEDS.out_bits.b0;
			if (V.blink & 0b00000100) LEDS.out_bits.b2 = !LEDS.out_bits.b2;
			if (V.blink & 0b00001000) LEDS.out_bits.b3 = !LEDS.out_bits.b2;
			if (V.blink & 0b00010000) LEDS.out_bits.b4 = !LEDS.out_bits.b4;
			if (V.blink & 0b00100000) LEDS.out_bits.b5 = !LEDS.out_bits.b4;
			if (V.blink & 0b01000000) LEDS.out_bits.b6 = !LEDS.out_bits.b6;
			if (V.blink & 0b10000000) LEDS.out_bits.b7 = !LEDS.out_bits.b6;
		} else {
			if (V.blink & 0b00000001) LEDS.out_bits.b0 = !LEDS.out_bits.b0;
			if (V.blink & 0b00000010) LEDS.out_bits.b1 = !LEDS.out_bits.b1;
			if (V.blink & 0b00000100) LEDS.out_bits.b2 = !LEDS.out_bits.b2;
			if (V.blink & 0b00001000) LEDS.out_bits.b3 = !LEDS.out_bits.b3;
			if (V.blink & 0b00010000) LEDS.out_bits.b4 = !LEDS.out_bits.b4;
			if (V.blink & 0b00100000) LEDS.out_bits.b5 = !LEDS.out_bits.b5;
			if (V.blink & 0b01000000) LEDS.out_bits.b6 = !LEDS.out_bits.b6;
			if (V.blink & 0b10000000) LEDS.out_bits.b7 = !LEDS.out_bits.b7;
		}
		if (LEDS.out_byte != led_cache || TRUE) {
			if (LEDS.out_bits.b1) {
				LED1 = LEDON;
			} else {
				LED1 = LEDOFF; // LED OFF
			}
			if (LEDS.out_bits.b2) {
				LED2 = LEDON;
			} else {
				LED2 = LEDOFF; // LED OFF
			}
			if (LEDS.out_bits.b3) {
				LED3 = LEDON;
			} else {
				LED3 = LEDOFF; // LED OFF
			}
			if (LEDS.out_bits.b4) {
				LED4 = LEDON;
			} else {
				LED4 = LEDOFF; // LED OFF
			}
			if (LEDS.out_bits.b5) {
				LED5 = LEDON;
			} else {
				LED5 = LEDOFF; // LED OFF
			}
			if (LEDS.out_bits.b6) {
				LED6 = LEDON;
			} else {
				LED6 = LEDOFF; // LED OFF
			}
			led_cache = LEDS.out_byte;
		}
		/* End Led Blink Code */
	}
	/*
	 * spurious interrupt handler
	 */
	if (!V.valid) {
		if (V.spurious_int++ > MAX_SPURIOUS)
			Reset();
	}

}

/* main loop routine */
int16_t sw_work(void)
{

	if (V.valid)
		ClrWdt(); // reset watchdog

	if (V.spinning) {
		blink_led(1, ON, ON); // LED blink
	} else {
		blink_led(1, ON, OFF); // LED STEADY
	}

	if (V.comm) {
		switch (V.comm_state) {
		case 1:
			putrsUSART(status1);
			break;
		case 2:
			putrsUSART(status2);
			break;
		case 3:
			putrsUSART(status3);
			itoa(V.spin_count, str);
			putsUSART(str);
			putrsUSART(spacer0);
			itoa(V.hi_rez_count, str);
			putsUSART(str);
			break;
		case 4:
			putrsUSART(status4);
			break;
		default:
			putrsUSART(status0);
			break;
		}
		V.comm = FALSE;
		V.sleep_ticks = 0;
	}
	/*
	 * shutdown the controller if nothing is happening
	 */
	if (V.sleep_ticks > SLEEP_COUNT) {
		LED1 = 1;
		LED2 = 1;
		RPMLED = 1;
		OSCCON = 0x00; // sleep, no clocks
		Sleep();
		OSCCON = 0x72;
	}

	return 0;
}

void init_rmsmon(void)
{
	/*
	 * check for a clean POR
	 */
	V.boot_code = FALSE;
	if (RCON != 0b0011100)
		V.boot_code = TRUE;

	if (STKPTRbits.STKFUL || STKPTRbits.STKUNF) {
		V.boot_code = TRUE;
		STKPTRbits.STKFUL = 0;
		STKPTRbits.STKUNF = 0;
	}

	OSCCON = 0x72;
	ADCON1 = 0x7F; // all digital, no ADC
	/* interrupt priority ON */
	RCONbits.IPEN = 1;
	/* define I/O ports */
	RMSPORTA = RMSPORT_IOA;
	RMSPORTB = RMSPORT_IOB;

	RPMOUT = LEDON; // preset all LEDS
	LED1 = LEDON;
	LED2 = LEDON;
	LED3 = LEDON;
	LED4 = LEDON;
	LED5 = LEDON;
	LED6 = LEDON;
	RPMLED = LEDON;
	Blink_Init();
	timer0_off = TIMEROFFSET; // blink fast
	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_64); // led blinker
	WriteTimer0(timer0_off); //	start timer0 at ~1/2 second ticks
	OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_4 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF); // Viision RPM signal
	WriteTimer1(SAMPLEFREQ);
	/* Light-link data input */
	//	COMM_ENABLE = TRUE; // for PICDEM4 onboard RS-232, not used on custom board
	OpenUSART(USART_TX_INT_OFF &
		USART_RX_INT_ON &
		USART_ASYNCH_MODE &
		USART_EIGHT_BIT &
		USART_CONT_RX, 50); // 8mhz internal osc 9600

	/*      work int thread setup */
	INTCONbits.TMR0IE = 1; // enable int
	INTCON2bits.TMR0IP = 1; // make it high level

	PIE1bits.TMR1IE = 1; // enable int
	IPR1bits.TMR1IP = 1; // make it high level

	INTCONbits.INT0IE = 1; // enable RPM sensor input
	INTCON2bits.RBPU = 0; // enable weak pull-ups

	init_rms_params();
}

/* give good spin signal at powerup */
uint8_t init_rms_params(void)
{
	V.sleep_ticks = 0;
	V.spin_count = RPM_COUNT;
	V.stop_tick = MAX_TICK;
	V.motor_ramp = START_RAMP;
	V.spinning = TRUE;
	V.sample_freq = SAMPLEFREQ;
	V.slew_freq = SAMPLEFREQ;
	V.valid = TRUE;
	V.spurious_int = 0;
	V.comm = FALSE;
	V.comm_state = 0;
	/* Enable all high priority interrupts */
	INTCONbits.GIEH = 1;
	putrsUSART(VER);
	putrsUSART(status1);
	putrsUSART(build_date);
	putrsUSART(spacer0);
	putrsUSART(build_time);
	putrsUSART(spacer1);
	if (V.boot_code) {
		itoa(RCON, str);
		putrsUSART(boot0);
		putsUSART(str);
		itoa(STKPTR, str);
		putrsUSART(boot1);
		putsUSART(str);
	}
	return 0;
}

void main(void)
{
	init_rmsmon();
	blink_led(2, ON, ON); // controller run indicator

	/* Loop forever */
	while (TRUE) { // busy work
		sw_work(); // run housekeeping
		Nop();
		Nop();
		Nop();
		Nop();
	}
}

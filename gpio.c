/*
 * GPIO.c 
 *
 * Taken mostly from http://elinux.org/RPi_Low-level_peripherals#GPIO_Driving_Example_.28C.29
 *
 * Adapted with a few extra comments to clarify those crazy macros
 */

#define BCM2708_PERI_BASE   0x20000000
#define GPIO_BASE	    (BCM2708_PERI_BASE + 0x200000) /* GPIO Controller */
#define PWM_BASE  	    (BCM2708_PERI_BASE + 0x20C000)  

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <time.h>
#include "dbg.h"

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int mem_fd;
char* gpio_map;
char* pwm_map;

volatile unsigned* gpio;
volatile unsigned* pwm;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
// FSEL starts at gpio and has 3 bits per pin, which allows (32/3),
// or 10 pins per 32-bit address. b000 = input, b001 = output, the alt appears
// to be manually specified by this script.  Alt functions start at b100 (0) and go
// up from there, so alt 1 = b101, alt 2 = b102, etc. 
#define FSEL_BANK(g)  ((g)/10)
#define FSEL_OFFSET(g) (((g)%10)*3)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)
#define GPIO_CLR *(gpio+10)

// offsets are divided by 4 because of pointer aritmetic on 32-bit pointers :)
#define GPIO_LEVEL(g) 		(*(gpio+(0x34/4)) & (1 << (g)))
#define RISING_EDGE_EN(g) 	*(gpio+(0x7C/4)) |= (1 << (g))
#define RISING_EDGE(g)		(*(gpio+(0x40/4)) & (1 << (g)))
#define CLR_RISING_EDGE(g)	*(gpio+(0x40/4)) |= (1 << (g))

/* PWM stuff */
#define PWM_CTL  	(*(pwm))
#define PWM_RANGE  	(*(pwm + (0x10/4)))
#define PWM_DATA 	(*(pwm + (0x14/4)))


void init_lcd();
void lcd_print(char* c, short line);
void setup_io();

void intHandler(int dummy) {
	GPIO_CLR = 1 << 7;  // turn it off
	SET_GPIO_ALT(18, 0);
	GPIO_CLR = 1 << 18;
	exit(1);
}

void init_pwm()
{
	INP_GPIO(18);
	SET_GPIO_ALT(18, 5); // enable PWM on GPIO 18
	PWM_CTL &= 1;

	PWM_DATA = 512;
	PWM_RANGE = 1024;

	//OUT_GPIO(18);
	//GPIO_SET = 1 << 18;
}


int main(int argc, char** argv)
{
	int g, rep;

	setup_io();
	signal(SIGINT, intHandler);

	debug("setting up io pins");
	INP_GPIO(17);
	OUT_GPIO(17);

	//init_pwm();

	int glow = 0;
	int increment = 4;
	
	while (0) {
		glow += increment;
		increment++;
		if (glow > 1024 || glow < 0) {
			glow -= increment;
			increment = -increment;
		}
		//if (GPIO_LEVEL(8)) {
			GPIO_SET = 1 << 17;
			PWM_DATA = glow;
		//} else {
		//	GPIO_CLR = 1 << 7;
		//}

		usleep(30000);
	}

	debug("initializing lcd");
	init_lcd();
	debug("writing to lcd");
	lcd_print("Raspberry Pi :)", 1);

	time_t t = time(NULL);
	struct tm cur_time = *localtime(&t);
	char time_str[19];

	while (1) {
		t = time(NULL);
		cur_time = *localtime(&t);
		strftime(time_str, 18, "%a %I:%M:%S %p", &cur_time);
		lcd_print(time_str, 2);
		sleep(1);
	}

	return 0;

	for (rep=0; rep<10; rep++) {
		for (g=7; g<=11; g++) {
			GPIO_SET = 1<<g;
			sleep(1);
		}
		//for (g=7; g<=11; g++) {
		//	GPIO_CLR = 1<<g;
		//	sleep(1);
		//}
	}

	return 0;
}

void setup_io()
{
	/* open /dev/mem */
	if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC) ) < 0) {
		printf("can't open /dev/mem\n");
		exit(-1);
	}

	/* mmap GPIO */
	gpio_map = (char*)mmap(
			NULL,
			BLOCK_SIZE,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
			mem_fd,
			GPIO_BASE
	);

	/*pwm_map = (char*)mmap(
			NULL,
			BLOCK_SIZE,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
			mem_fd,
			PWM_BASE
	);*/

	close(mem_fd);

	if (gpio_map == MAP_FAILED) {
		printf("mmap error %d\n", (int)gpio_map);
		exit(-1);
	}
	
	if ((long)pwm_map < 0) {
		printf("mmap error for pwm %d\n", (int)gpio_map);
		exit(-1);
	}

	/* I believe this is because reads can happen out-of-order for peripherals */
	gpio = (volatile unsigned *)gpio_map;
	pwm  = (volatile unsigned *)pwm_map;
}

#define DB4 	7   
#define DB5	24
#define DB6	9
#define DB7	10
#define RS	11
#define E	25

#define SET_OUTP(g, i)   \
	if (i) { GPIO_SET = (1 << (g)); } \
	else { GPIO_CLR = (1 << (g)); }

void write_output(char i)
{
	i >>= 4;
	debug("writing %x", i);
	SET_OUTP(DB4, (i & 1));
	SET_OUTP(DB5, (i & 2));
	SET_OUTP(DB6, (i & 4));
	SET_OUTP(DB7, (i & 8));
	//sleep(3);
}

void nybble()
{
	SET_OUTP(E, 1);
	usleep(1000);
	SET_OUTP(E, 0);
}

void command(char i)
{
	debug("command 0x%x", i);
	SET_OUTP(RS, 0);
	write_output(i);
	nybble();
	i <<= 4;
	write_output(i);
	nybble();	
}

void lcd_write(char i)
{
	debug("writing character %c", i);
	SET_OUTP(RS, 1);
	write_output(i);
	nybble();
	i <<= 4;
	write_output(i);
	nybble();
}

void lcd_print(char* str, short line)
{
	command(0x80 | (line == 2 ? 0x40 : 0));

	while (*str) {
		lcd_write(*str++);
	}
}

#define SETUP_OUT_PIN(g)  INP_GPIO(g); OUT_GPIO(g);

void init_lcd()
{
	SETUP_OUT_PIN(DB4);
	SETUP_OUT_PIN(DB5);
	SETUP_OUT_PIN(DB6);
	SETUP_OUT_PIN(DB7);
	SETUP_OUT_PIN(RS);
	SETUP_OUT_PIN(E);

	SET_OUTP(E, 0);
	SET_OUTP(RS, 0);
	write_output(0);
	//SET_OUTP(P3, 0);
	usleep(100000);
	write_output(0x30); // wake up
	usleep(30000);
	nybble();
	usleep(10000);
	nybble();
	usleep(10000);
	nybble();
	usleep(10000);
	sleep(1);
	nybble();
	write_output(0x20);  // 4-bit interface
	nybble();
	command(0x28);    // 4-bit / 2 lines 
	command(0x10);	  // set cursor
	command(0x0C);	  // display on, blinking cursor
	command(0x01);
	usleep(2000);
	command(0x02);
	usleep(2000);
	command(0x06);    // entry mode set
}


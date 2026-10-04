// MHO900 PWM fan controller.
// Tool to automatically and manually control PWM fan in the Rigol MHO900 series oscilloscopes, read temperatures (-r) and power off (-s).
// Use -h switch to get full help about usage.
// 
// Version: 2.1
//
// NOTE: controlling the fan will not work fully properly with the stock app from the Rigol update v00.01.00.00.26, because toghether this and the app periodically will change PWM value in the kernel module, which will cause fan speed oscillations - app once per two seconds (in majority of cases always to 77) and this much more often. Older stock Rigol scope apps may change PWM value only once in some rare occasions, which is not a problem - in practice fan may go full speed for about one second.
//
// Copyright (C) 2026 Norbert Kiszka
// 
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; version 2
// of the License.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// 
// See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

// SPDX-License-Identifier: GPL-2.0


#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/ioctl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <signal.h>
#include <getopt.h>

#define DEBUG_TEMPERATURE_FILE_READ 0
#define DEBUG_PWM_SET 0

#define PWM_MIN 75
#define PWM_MANUAL_MIN 180

#define DEFAULT_TEMP_FAN_OFF 35
#define DEFAULT_TEMP_MIN 45
#define DEFAULT_TEMP_MAX 65
#define EMERGENCY_SHUTDOWN_TEMP 90


#define MHO900_FAN_VER 2.1
#define FILE_FAN "/dev/pwm_fan"
#define FILE_TEMP0 "/sys/devices/virtual/thermal/thermal_zone0/temp"
#define FILE_TEMP1 "/sys/devices/virtual/thermal/thermal_zone1/temp"
#define FILE_UART "/dev/ttyS0"
#define IOCTL_CMD_PWM_SET 0x7801
#define IOCTL_CMD_PWM_GET 0x7802

#define GETMAXFROM2(a, b) a > b ? a : b
#define GETMINFROM2(a, b) a < b ? a : b
#define S_(x) #x
#define S(x) S_(x)
#define TEMP2FLOAT(val) (float)((float)val * 0.001)
#define PWM_DIFF (255 - PWM_MIN)

#define GET_TEMP0() get_temp(FILE_TEMP0)
#define GET_TEMP1() get_temp(FILE_TEMP1)

#define GET_TEMP0_FLOAT() TEMP2FLOAT(GET_TEMP0())
#define GET_TEMP1_FLOAT() TEMP2FLOAT(GET_TEMP1())

// Value returned in case of unlikely read or parse problems. It must remain high to prevent accidental overheat and must be below EMERGENCY_SHUTDOWN_TEMP (multipled by 1000) to prevent inadvertent emergency shutdown.
#define READ_FAIL_TEMP (EMERGENCY_SHUTDOWN_TEMP - 1) * 1000

static int fd_pwm = -1;
static int8_t do_prints = 0;

static int32_t get_temp(char *file)
{
	ssize_t bytes_read;
	char temp_buffer[8];
	int32_t ret;
	
	// We need to open it for every single read.
	int fd_temp = open(file, O_RDONLY);
	
	if(fd_temp < 0)
	{
		fprintf(stderr, "Failed to open temperature file: %s\n", file);
		return READ_FAIL_TEMP;
	}
	
	temp_buffer[0] = 0;
	
	bytes_read = read(fd_temp, temp_buffer, sizeof(temp_buffer) - 1);
	
	close(fd_temp);
	
	if(bytes_read > 0)
		temp_buffer[bytes_read - 1] = 0; // Change LF to NULL.
		
	temp_buffer[ sizeof(temp_buffer) - 1 ] = 0; // Make sure that buffer last byte is always NULL.
	
#if DEBUG_TEMPERATURE_FILE_READ
	printf("File \"%s\" - %zu bytes, data: %s.\n", file, bytes_read, temp_buffer);
#endif
	
	if(bytes_read != 6 && bytes_read != 7)
	{
		fprintf(stderr, "Improper read result of a temperature file: %s, read() returned %zu bytes.\n", file, bytes_read);
		return READ_FAIL_TEMP;
	}
	
	ret = atoi(temp_buffer);
	
	if(ret <= 0)
	{
		fprintf(stderr, "Temperature acquired from the kernel seems to be 0 or lower. atoi() returned: %i read() buffer: %s\n", ret, temp_buffer);
		return READ_FAIL_TEMP; // No scopes in Syberia.
	}
	
	return ret;
}

static int32_t set_pwm(int v)
{
	int32_t ret = 0;
#if DEBUG_PWM_SET
	int32_t pwm_read;
#endif
	
#if !DEBUG_PWM_SET
	if(do_prints)
	{
#endif
		printf("PWM: %i\n", v);
#if !DEBUG_PWM_SET
	}
#endif
	
	if(fd_pwm < 0)
	{
		fd_pwm = open(FILE_FAN, O_RDWR);
		if(fd_pwm < 0)
		{
			fprintf(stderr, "Failed to open PWM device: " S(FILE_FAN) "\n");
			usleep(0xF0000); // In case of permament error, printing too often may be very very bad.
			return 1;
		}
	}
	
	if(ioctl(fd_pwm, IOCTL_CMD_PWM_SET, v) < 0)
	{
		fprintf(stderr, "Ioctl set failed :/\n");
		ret = 1;
		usleep(0xF0000);
	}
	
#if DEBUG_PWM_SET
	if(ioctl(fd_pwm, IOCTL_CMD_PWM_GET, &pwm_read) < 0)
	{
		fprintf(stderr, "Ioctl read failed...\n");
		usleep(20000);
	}
	else
	{
		printf("PWM READ: %i (%s)\n", pwm_read, pwm_read == v ? "OK" : "DIFFERENT");
		
		if(pwm_read != v)
		{
			ret = 1;
			usleep(0xF0000);
		}
	}
#endif
	
	return ret;
}

static void signal_handler(int sig)
{
	printf("Received signal %i.\n", sig);
	set_pwm(255);
	usleep(0xF0000); // In case of unlikely infinite loop.
	_exit(sig);
}

static void usage(char *progname)
{
	printf("Usage:\n\n");
	if(progname)
		printf("%s\n%s [optional args]\n\n", progname, progname);
	
	printf(
	"\t-o #\tTemperature treshold to turn off the fan. Must be between 5 and 50. Default: " S(DEFAULT_TEMP_FAN_OFF) ".\n"
	"\t-m #\tTemperature treshold/level to run fan at the minimum. Must be higher than off threshold by at least 5. Default: " S(DEFAULT_TEMP_MIN) ".\n"
	"\t-M #\tTemperature level to run fan at the maximum. Must be higher than min threshold by at least 5 and can't be higher than 75. Default: " S(DEFAULT_TEMP_MAX) ".\n"
	"\n\t-p\tPrint temperature thresholds.\n"
	"\n\t-d\tPrint temperature and PWM value (0-255) at every single PWM change.\n"
	"\n\t-r\tRead temperatures and exit.\n"
	"\t-e #\tSet PWM fan value (" S(PWM_MANUAL_MIN) "-255) and exit.\n"
	"\t-s #\tSwitch off the device within given amount of seconds.\n"
	"\t-h\tPrint full help and exit.\n"
	"\nAll temperatures are in Celsius. All arg parameters are integer only (full number without dot or comma).\n"
	"\nCaution: higher temperatures for a prolonged time may significantly reduce life time of the device. It's highly recommended to keep temperatures below 60.\n"
	);
}

static inline void help(char *progname)
{
	printf("\nMHO900 PWM fan controller by NK. Version: " S(MHO900_FAN_VER) "\n\n");
	usage(progname);
	printf("\nExamples:\n"
	"%s -o 40 -m 50 -M 75\n"
	"%s -o 20 -m 30 -M 55 -d\n"
	"%s -r -e 200\n\n"
	, progname, progname, progname);
}

#define SHUTDOWN_ATTEMPTS 20
static void shutdown(void)
{
	int fd, i;
	
	for(i = 1; i <= SHUTDOWN_ATTEMPTS; i++)
	{
		fd = open(FILE_UART, O_WRONLY);
		if(fd > -1)
		{
			write(fd, "\xfa\x05\x01\x2e\xaf", 5); // There is extremely low chance for write() failure. In such case something is screwed up totally and either You have random flipped bits on the SD card or You should throw Your scope out of the window.
			usleep(0x30000); // After this, scope should be switched off long time ago, so the very next line shouldn't be executed.
			fprintf(stderr, "Device shutdown failed. Attempt %i/" S(SHUTDOWN_ATTEMPTS) "\n", i);
			close(fd);
		}
		else
		{
			fprintf(stderr, "Failed to open UART device %s needed to power off the scope. Attempt %i/" S(SHUTDOWN_ATTEMPTS) "\n", FILE_UART, i); // Hit the scope couple times. It often helps to fix cold joints for some time.
			usleep(0xf0000); // Hopefully this is enough time to make it fixed by itself just by looking at it.
		}
	}
}

// Separate function mainly to reduce compiled code in the main_loop().
static void emergency_shutdown(int32_t tempmax)
{
	fprintf(stderr, "Emergency shutdown due to critical temperature: %.3f >= " S(EMERGENCY_SHUTDOWN_TEMP) "\n", TEMP2FLOAT(tempmax));
	shutdown();
	while(1) // Just in case.
	{
		set_pwm(255);
		usleep(0x200000);
	}
}

static void main_loop(int32_t setting_temp_fan_off, float f_setting_temp_fan_min, float f_setting_temp_fan_max, int32_t setting_temp_fan_max)
{
	int32_t tempmax, t;
	int32_t reads[32];
	int32_t pwm;
	int32_t temp;
	int8_t i;
	int32_t readpointer = 0;
	int32_t prev_pwm = 999; // prev_pwm should be initialized with any value outside of the range 0-255.
	
	// Reset the whole array. Fan most likely at this point is running at the full blast.
	for(i = 0; i <= 31; i++)
	{
		reads[i] = setting_temp_fan_max;
	}
	
	while(1)
	{
		//tempmax = GETMAXFROM2(GET_TEMP0(), GET_TEMP1());
		tempmax = GET_TEMP0();
		t = GET_TEMP1();
		tempmax = GETMAXFROM2(tempmax, t);
		
		if(tempmax >= (EMERGENCY_SHUTDOWN_TEMP * 1000))
		{
			emergency_shutdown(tempmax);
		}
		
		reads[readpointer] = tempmax;
		
		temp = 0;
		for(i = 0; i <= 31; i++)
		{
			temp += reads[i]; // Compiler should use vector instructions here.
		}
		
		temp = temp >> 5;
		
		if(temp < setting_temp_fan_off)
		{
			pwm = 0;
		}
		else
		{
			pwm = (int32_t)(PWM_MIN + ( (TEMP2FLOAT(temp) - f_setting_temp_fan_min) / (f_setting_temp_fan_max - f_setting_temp_fan_min) ) * PWM_DIFF); // Black magic.
			
			if(pwm < PWM_MIN)
			{
				pwm = PWM_MIN;
			}
			if(pwm > 255)
			{
				pwm = 255;
			}
		}
		
		if(pwm != prev_pwm)
		{
			if(do_prints)
			{
				printf("AVG TEMP: %.2f\n", TEMP2FLOAT(temp)); // PWM value is printed in set_pwm().
			}
			
			if(set_pwm(pwm) == 0)
			{
				prev_pwm = pwm; // Change prev_pwm only when PWM was succesfully set by ioctl. Otherwise, it will try again in the very next loop.
			}
			
			if(do_prints)
			{
				printf("---------------------------\n");
			}
		}
		
		usleep(0xF000); // 2s divided by 32. Rounded in hex, to reduce amount of CPU instructions (movk). Note: 3s makes significantly bigger oscillations.
		
		readpointer++;
		
		if(readpointer >= 32)
		{
			readpointer = 0;
		}
	}
}

int main(int argc, char *argv[])
{
	int32_t option, t, ret = 0, set_pwm_manual = -1, shutdown_seconds = -1;
	int8_t do_exit = 0, print_settings = 0;
	
	int32_t setting_temp_fan_off = DEFAULT_TEMP_FAN_OFF * 1000;
	int32_t setting_temp_fan_min = DEFAULT_TEMP_MIN * 1000;
	int32_t setting_temp_fan_max = DEFAULT_TEMP_MAX * 1000;
	
	while((option = getopt(argc, argv, "dhro:m:M:e:ps:")) != -1)
	{
		switch (option)
		{
			case 'h':
				help(argv[0]);
				_exit(0);
			break;
			
			case 'e':
				set_pwm_manual = atoi(optarg);
				if(set_pwm_manual < 0)
				{
					set_pwm_manual = 0;
				}
			break;
			
			case 'r':
				printf
				(
					"TEMP 0: %.2f\n"
					"TEMP 1: %.2f\n"
					, GET_TEMP0_FLOAT(), GET_TEMP1_FLOAT()
				);
				do_exit = 1;
			break;
			
			case 'd':
				do_prints = 1;
			break;
			
			case 'o':
				t = atoi(optarg);
				
				if(t < 0)
				{
					fprintf(stderr, "Fan switch off temp must be at 5 or higher. Changing it to 5.\n");
					t = 5;
				}
				
				if(t > 50)
				{
					fprintf(stderr, "Fan switch off temp can't be higher than 50. Changing it to 50.\n");
					t = 50;
				}
				setting_temp_fan_off = t * 1000;
			break;
			
			case 'm':
				t = atoi(optarg);
				setting_temp_fan_min = t * 1000;
			break;
			
			case 'M':
				t = atoi(optarg);
				setting_temp_fan_max = t * 1000;
			break;
			
			case 'p':
				print_settings = 1;
			break;
			
			case 's':
				shutdown_seconds = atoi(optarg);
				if(shutdown_seconds <= 0)
				{
					shutdown_seconds = 0;
				}
			break;
			
			case ':':
			case '?':
				usage(argv[0]);
				do_exit = 1;
			break;
			
			default:
				fprintf(stderr, "Unhandled option or getopt returned impossible value: %d ('%c').\n", option, option);
				do_exit = 1;
		}
	}
	
	// if(print_settings)
	// {
	// 	printf
	// 	(
	// 		"Off threshold: %i\n"
	// 		"Min threshold: %i\n"
	// 		"Max threshold: %i\n"
	// 	, setting_temp_fan_off / 1000, setting_temp_fan_min / 1000, setting_temp_fan_max / 1000);
	// }
	
	if(set_pwm_manual > -1)
	{
		if(set_pwm_manual < PWM_MANUAL_MIN)
		{
			fprintf(stderr, "!!! Given PWM value is lower than allowed minimum of " S(PWM_MANUAL_MIN) ". Changing it to " S(PWM_MANUAL_MIN) " !!!\n");
			set_pwm_manual = PWM_MANUAL_MIN;
		}
		if(set_pwm_manual > 255)
		{
			fprintf(stderr, "Given PWM value exceeds maximum possible 255. Changing it to 255.\n");
			set_pwm_manual = 255;
		}
		ret = set_pwm(set_pwm_manual);
		if(fd_pwm > -1)
		{
			close(fd_pwm);
		}
		do_exit = 1;
	}
	
	if(shutdown_seconds >= 0)
	{
		if(shutdown_seconds > 0)
		{
			while(shutdown_seconds > 0)
			{
				printf("%i\n", shutdown_seconds);
				usleep(0xF4000);
				shutdown_seconds--;
			}
		}
		shutdown();
		_exit(1);
	}
	
	if(do_exit)
	{
		_exit(ret);
	}
	
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGFPE, signal_handler);
	signal(SIGTSTP, signal_handler);
	
	if(setting_temp_fan_max > 75000)
	{
		fprintf(stderr, "Max temp. threshold can't be higher than 75. Reducing it to 75.\n");
		setting_temp_fan_max = 75000;
	}
	
	if(setting_temp_fan_min > setting_temp_fan_max - 5000)
	{
		setting_temp_fan_min = setting_temp_fan_max - 5000;
		fprintf(stderr, "Min temp. threshold must be lower than max temp. threshold by at least 5. Reducing it to %i.\n", setting_temp_fan_min / 1000);
	}
	
	if(setting_temp_fan_off > setting_temp_fan_min - 5000)
	{
		setting_temp_fan_off = setting_temp_fan_min - 5000;
		fprintf(stderr, "Off fan temp. threshold must be lower than min temp. threshold by at least 5. Reducing it to %i.\n", setting_temp_fan_off / 1000);
	}
	
	if(print_settings)
	{
		printf
		(
			"Off threshold: %i\n"
			"Min threshold: %i\n"
			"Max threshold: %i\n"
		, setting_temp_fan_off / 1000, setting_temp_fan_min / 1000, setting_temp_fan_max / 1000);
	}
	
	main_loop(setting_temp_fan_off, TEMP2FLOAT(setting_temp_fan_min), TEMP2FLOAT(setting_temp_fan_max), setting_temp_fan_max); // Compiler should ignore everything after this line, because it contains infinite loop.
	
	set_pwm(255); // Must be the wind.
	printf("You shouldn't see this message...\n");
	return 1;
}

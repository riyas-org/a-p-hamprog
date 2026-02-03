/* **************************************************************************
 * Project: pp3r - PIC Programmer for ATU-100
 * Author:  lb7ug
 * * Credits: Based on the original a-p-prog by Jaromir Sukuba 
 *
 * Purpose: A quick-n-dirty weekend hobby project specifically made for 
 * programming PICs in the ATU-100 antenna tuner (PIC16F1938).
 * * ⚠️ DISCLAIMER:
 * - NO GUARANTEE: This is experimental software. Use at your own risk.
 * - VARIANT ISSUES: May not handle all PIC variants correctly.
 * - HEX DUMP WARNING: Dumping/Reading hex files may have issues with 
 * Config Bytes. These often require manual correction in the HEX file.
 *
 * 🛠 CONTRIBUTIONS:
 * This is a hobbyist tool—feel free to modify, optimize, and improve! 
 * Pull requests on GitHub are highly encouraged.
 * https://github.com/riyas-org/a-p-hamprog 
 * ************************************************************************** */

#include <ctype.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#if defined(__linux__) || defined(__APPLE__)
#include <termios.h>
#else
#include <windows.h>
#endif

int setCPUtype(char *cpu);
int parse_hex(char *filename, unsigned char *buffer); // Fixed: 2 arguments
size_t getlinex(char **lineptr, size_t *n, FILE *stream);
void comErr(char *fmt, ...);
void flsprintf(FILE *f, char *fmt, ...);

char *COM = "";
char *PP_VER = "1.0-lb7ug";
char *hex_out_name = "dump.hex";

#define PROGMEM_LEN 260000
#define CONFIG_LEN 32


#define CF_P16F_A 0
#define CF_P18F_A 1
#define CF_P16F_B 2
#define CF_P18F_B 3
#define CF_P18F_C 4
#define CF_P18F_D 5
#define CF_P18F_E 6
#define CF_P16F_C 7
#define CF_P16F_D 8
#define CF_P18F_F 9
#define CF_P18F_G 10
#define CF_P18F_Q 11

int verbose = 1, verify = 1, program = 1, sleep_time = 0, readflash = 0;
int hv_programming = 0;
int devid_expected, devid_mask, baudRate, com, flash_size, page_size,
    chip_family, config_size;
unsigned char file_image[70000], progmem[PROGMEM_LEN], config_bytes[CONFIG_LEN];

//*********************************************************************************//
//*********************************************************************************//
//*********************************************************************************//
//               serial IO interfaces for Linux and windows
//*********************************************************************************//
//*********************************************************************************//
//*********************************************************************************//

#if defined(__linux__) || defined(__APPLE__)

void initSerialPort() {
	baudRate = B57600;
	if (verbose > 2)
		printf("Opening: %s at %d\n", COM, baudRate);
	com = open(COM, O_RDWR | O_NOCTTY | O_NDELAY);
	if (com < 0)
		comErr("Failed to open serial port");

	struct termios opts;
	memset(&opts, 0, sizeof(opts));

	fcntl(com, F_SETFL, 0);
	if (tcgetattr(com, &opts) != 0)
		printf("Err tcgetattr\n");

	cfsetispeed(&opts, baudRate);
	cfsetospeed(&opts, baudRate);
	opts.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	opts.c_cflag |= (CLOCAL | CREAD);
	opts.c_cflag &= ~PARENB;
	opts.c_cflag &= ~CSTOPB;
	opts.c_cflag &= ~CSIZE;
	opts.c_cflag |= CS8;
	opts.c_oflag &= ~OPOST;
	opts.c_iflag &= ~INPCK;
	opts.c_iflag &= ~ICRNL; // do NOT translate CR to NL
	opts.c_iflag &= ~(IXON | IXOFF | IXANY);
	opts.c_cc[VMIN] = 0;
	opts.c_cc[VTIME] = 10; // 0.1 sec
	if (tcsetattr(com, TCSANOW, &opts) != 0) {
		perror(COM);
		printf("set attr error");
		abort();
	}
	tcflush(com, TCIOFLUSH); // just in case some crap is the buffers
}

void putByte(int byte) {
	char buf = byte;
	if (verbose > 3)
		flsprintf(stdout, "TX: 0x%02X\n", byte);
	int n = write(com, &buf, 1);
	if (n != 1)
		comErr("Serial port failed to send a byte, write returned %d\n", n);
}

void putBytes(unsigned char *data, int len) {

	int i;
	for (i = 0; i < len; i++)
		putByte(data[i]);
	/*
	    if (verbose>3)
	            flsprintf(stdout,"TXP: %d B\n", len);
	int n = write(com, data, len);
	    if (n != len)
	            comErr("Serial port failed to send %d bytes, write returned %d\n",
	len,n);
	*/
}

int getByte() {
	char buf;
	int n = read(com, &buf, 1);
	if (verbose > 3)
		flsprintf(stdout, n < 1 ? "RX: fail\n" : "RX:  0x%02X\n", buf & 0xFF);
	if (n == 1)
		return buf & 0xFF;

	comErr("Serial port failed to receive a byte, read returned %d\n", n);
	return -1; // never reached
}
#else

HANDLE port_handle;

void initSerialPort() {

	char mode[40], portname[20];
	COMMTIMEOUTS timeout_sets;
	DCB port_sets;
	strcpy(portname, "\\\\.\\");
	strcat(portname, COM);
	port_handle =
	    CreateFileA(portname, GENERIC_READ | GENERIC_WRITE, 0, /* no share  */
	                NULL,                                      /* no security */
	                OPEN_EXISTING, 0,                          /* no threads */
	                NULL);                                     /* no templates */
	if (port_handle == INVALID_HANDLE_VALUE) {
		printf("unable to open port %s -> %s\n", COM, portname);
		exit(0);
	}
	strcpy(mode, "baud=57600 data=8 parity=n stop=1");
	memset(&port_sets, 0, sizeof(port_sets)); /* clear the new struct  */
	port_sets.DCBlength = sizeof(port_sets);

	if (!BuildCommDCBA(mode, &port_sets)) {
		printf("dcb settings failed\n");
		CloseHandle(port_handle);
		exit(0);
	}

	if (!SetCommState(port_handle, &port_sets)) {
		printf("cfg settings failed\n");
		CloseHandle(port_handle);
		exit(0);
	}

	timeout_sets.ReadIntervalTimeout = 1;
	timeout_sets.ReadTotalTimeoutMultiplier = 1000;
	timeout_sets.ReadTotalTimeoutConstant = 1;
	timeout_sets.WriteTotalTimeoutMultiplier = 1000;
	timeout_sets.WriteTotalTimeoutConstant = 1;

	if (!SetCommTimeouts(port_handle, &timeout_sets)) {
		printf("timeout settings failed\n");
		CloseHandle(port_handle);
		exit(0);
	}
}
void putByte(int byte) {
	int n;
	if (verbose > 3)
		flsprintf(stdout, "TX: 0x%02X\n", byte);
	WriteFile(port_handle, &byte, 1, (LPDWORD)((void *)&n), NULL);
	if (n != 1)
		comErr("Serial port failed to send a byte, write returned %d\n", n);
}

void putBytes(unsigned char *data, int len) {
	/*
	int i;
	for (i=0;i<len;i++)
	    putByte(data[i]);
	*/
	int n;
	WriteFile(port_handle, data, len, (LPDWORD)((void *)&n), NULL);
	if (n != len)
		comErr("Serial port failed to send a byte, write returned %d\n", n);
}

int getByte() {
	unsigned char buf[2];
	int n;
	ReadFile(port_handle, buf, 1, (LPDWORD)((void *)&n), NULL);
	if (verbose > 3)
		flsprintf(stdout, n < 1 ? "RX: fail\n" : "RX:  0x%02X\n", buf[0] & 0xFF);
	if (n == 1)
		return buf[0] & 0xFF;
	comErr("Serial port failed to receive a byte, read returned %d\n", n);
	return -1; // never reached
}
#endif

//*********************************************************************************//
//*********************************************************************************//
//*********************************************************************************//
//               generic routines
//*********************************************************************************//
//*********************************************************************************//
//*********************************************************************************//

void write_hex_line(FILE *f, int len, int addr, int type, unsigned char *data) {
	unsigned char checksum = (unsigned char)len + (unsigned char)(addr >> 8) +
	                         (unsigned char)(addr & 0xFF) + (unsigned char)type;
	fprintf(f, ":%02X%04X%02X", len, addr & 0xFFFF, type);
	for (int i = 0; i < len; i++) {
		fprintf(f, "%02X", data[i]);
		checksum += data[i];
	}
	fprintf(f, "%02X\n", (unsigned char)(-checksum));
}

void comErr(char *fmt, ...) {
	char buf[500];
	va_list va;
	va_start(va, fmt);
	vsnprintf(buf, sizeof(buf), fmt, va);
	fprintf(stderr, "%s", buf);
	perror(COM);
	va_end(va);
	exit(-1);
}

void flsprintf(FILE *f, char *fmt, ...) {
	char buf[500];
	va_list va;
	va_start(va, fmt);
	vsnprintf(buf, sizeof(buf), fmt, va);
	fprintf(f, "%s", buf);
	fflush(f);
	va_end(va);
}

int is_empty(unsigned char *buff, int len) {
	int i, empty;
	empty = 1;
	for (i = 0; i < len; i++)
		if (buff[i] != 0xFF)
			empty = 0;
	return empty;
}

// get line replacement
size_t getlinex(char **lineptr, size_t *n, FILE *stream) {
	char *bufptr = NULL;
	char *p = bufptr;
	size_t size;
	int c;

	if (lineptr == NULL)
		return -1;
	if (stream == NULL)
		return -1;
	if (n == NULL)
		return -1;
	bufptr = *lineptr;
	size = *n;

	c = fgetc(stream);
	if (c == EOF)
		return -1;
	if (bufptr == NULL) {
		bufptr = malloc(128);
		if (bufptr == NULL) {
			return -1;
		}
		size = 128;
	}
	p = bufptr;
	while (c != EOF) {
		if ((p - bufptr) > (size - 1)) {
			size = size + 128;
			bufptr = realloc(bufptr, size);
			if (bufptr == NULL) {
				return -1;
			}
		}
		*p++ = c;
		if (c == '\n') {
			break;
		}
		c = fgetc(stream);
	}
	*p++ = '\0';
	*lineptr = bufptr;
	*n = size;
	return p - bufptr - 1;
}

void sleep_ms(int num) {
#if defined(__linux__)
	struct timespec tspec;
	tspec.tv_sec = num / 1000;
	tspec.tv_nsec = (num % 1000) * 1000000;
	nanosleep(&tspec, 0);
#else
	Sleep(num);
#endif
}

void printHelp() {
	printf("A-P-Prog+ v%s (lb7ug Edition)\n", PP_VER);
	printf("Tweaked for PIC16(L)F1938 (ATU-100, etc.)\n");
	printf("----------------------------------------------------------\n");
	printf("Original Engine: Jaromir Sukuba\n");
	printf("HVP & Read-to-HEX & quick n dirty fixes: lb7ug\n\n");
	printf("Usage: pp3r -c <port> -t <cpu> [options]\n\n");
	printf("Options:\n");
	printf("  -c <port>    Serial port (COMx or /dev/ttyUSBx)\n");
	printf("  -t <cpu>     Target MCU (e.g., 16f1938)\n");
	printf("  -r [name]    Read to Intel HEX (Default: dump.hex)\n");
	printf("  -h           High Voltage Mode (Requires 9.5V HVP hardware)\n");
	printf("  -v <0-2>     Verbosity level\n\n");
	printf("Note: HVP mode requires an external boost converter + optocoupler\n");
	printf("circuit to provide ~9.5V to the MCLR pin.\n");
	printf("Many ebay/aliexpress atu100 units have codeprotection and HVP enabled\n");
	printf("So at least once you need to program a firmware with LVP config using HVP\n");
	printf("Only High Voltage mode can turn on/off the HVP config bit\n");
	exit(0);
}

void parseArgs(int argc, char *argv[]) {
	int c;
	// while ((c = getopt (argc, argv, "c:nps:t:v:b:r::h")) != -1)
	while ((c = getopt(argc, argv, "c:nprs:t:v:b:h")) != -1) {
		switch (c) {
		case 'c':
			COM = optarg;
			break;
		case 'n':
			verify = 0;
			break;
		case 'p':
			program = 0;
			break;
		case 'r':
			readflash = 1;
			// if (optarg) hex_out_name = optarg;
			if (optind < argc && argv[optind][0] != '-') {
				hex_out_name = argv[optind];
				optind++;
			}
			break;
		case 's':
			sscanf(optarg, "%d", &sleep_time);
			break;
		case 't':
			setCPUtype(optarg);
			break;
		case 'v':
			sscanf(optarg, "%d", &verbose);
			break;
		case 'b':
			sscanf(optarg, "%d", &baudRate);
			break;
		case 'h':
			hv_programming =
			    1; // You must declare 'int hv_programming = 0;' at top of main
			printf("******** High Voltage Mode activated ******  \n");
			break;
		case '?':
			if (isprint(optopt))
				fprintf(stderr, "Unknown option `-%c'.\n", optopt);
			else
				fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
		default:
			fprintf(stderr, "Bug, unhandled option '%c'\n", c);
			// abort ();
			exit(0);
		}
	}
	if (argc <= 1)
		printHelp();
}

//*********************************************************************************//
//*********************************************************************************//
//*********************************************************************************//
//               programming routines
//*********************************************************************************//
//*********************************************************************************//
//*********************************************************************************//

int setCPUtype(char *cpu) {
	int name_len, i, read;
	name_len = strlen(cpu);
	for (i = 0; i < name_len; i++)
		cpu[i] = tolower(cpu[i]);
	char *line = NULL;
	char *filename = "pp3_devices.dat";
	char read_cpu_type[20], read_algo_type[20];
	int read_flash_size, read_page_size, read_id, read_mask;

	size_t len = 0;
	if (verbose > 2)
		printf("Opening filename %s \n", filename);
	FILE *sf = fopen(filename, "r");
	if (sf == 0) {
		return -1;
		if (verbose > 0)
			printf("Can't open database file %s\n", filename);
	}
	if (verbose > 2)
		printf("File open\n");
	while ((read = getlinex(&line, &len, sf)) != -1) {
		if (verbose > 3)
			printf("\nRead %d chars: %s", read, line);
		if (line[0] != '#') {
			sscanf(line, "%s %d %d %x %x %s", (char *)&read_cpu_type,
			       &read_flash_size, &read_page_size, &read_id, &read_mask,
			       (char *)&read_algo_type);
			if (verbose > 3)
				printf("\n*** %s,%d,%d,%x,%x,%s", read_cpu_type, read_flash_size,
				       read_page_size, read_id, read_mask, read_algo_type);
			if (strcmp(read_cpu_type, cpu) == 0) {
				flash_size = read_flash_size;
				page_size = read_page_size;
				devid_expected = read_id;
				devid_mask = read_mask;
				if (verbose > 1)
					printf("Found database match %s,%d,%d,%x,%x,%s\n", read_cpu_type,
					       read_flash_size, read_page_size, read_id, read_mask,
					       read_algo_type);
				if (strcmp("CF_P16F_A", read_algo_type) == 0)
					chip_family = CF_P16F_A;
				if (strcmp("CF_P16F_B", read_algo_type) == 0)
					chip_family = CF_P16F_B;
				if (strcmp("CF_P16F_C", read_algo_type) == 0)
					chip_family = CF_P16F_C;
				if (strcmp("CF_P16F_D", read_algo_type) == 0)
					chip_family = CF_P16F_D;
				if (strcmp("CF_P18F_A", read_algo_type) == 0)
					chip_family = CF_P18F_A;
				if (strcmp("CF_P18F_B", read_algo_type) == 0)
					chip_family = CF_P18F_B;
				if (strcmp("CF_P18F_C", read_algo_type) == 0)
					chip_family = CF_P18F_C;
				if (strcmp("CF_P18F_D", read_algo_type) == 0)
					chip_family = CF_P18F_D;
				if (strcmp("CF_P18F_E", read_algo_type) == 0)
					chip_family = CF_P18F_E;
				if (strcmp("CF_P18F_F", read_algo_type) == 0)
					chip_family = CF_P18F_F;
				if (strcmp("CF_P18F_G", read_algo_type) == 0)
					chip_family = CF_P18F_G;
				if (strcmp("CF_P18F_Q", read_algo_type) == 0)
					chip_family = CF_P18F_Q;
				if (chip_family == CF_P18F_A)
					config_size = 16;
				if (chip_family == CF_P18F_B)
					config_size = 8;
				if (chip_family == CF_P18F_C) {
					config_size = 16;
					chip_family = CF_P18F_B;
				}
				if (chip_family == CF_P18F_D)
					config_size = 16;
				if (chip_family == CF_P18F_E)
					config_size = 16;
				if (chip_family == CF_P18F_F)
					config_size = 12;
				if (chip_family == CF_P18F_Q)
					config_size = 12;
				if (chip_family == CF_P18F_G) {
					config_size = 10;
					chip_family = CF_P18F_F;
				}
				if (verbose > 2)
					printf("chip family:%d, config size:%d\n", chip_family, config_size);
			}
		}
	}
	fclose(sf);
	return 0;
}

int p16a_rst_pointer(void) {
	if (verbose > 2)
		flsprintf(stdout, "Resetting PC\n");
	if (chip_family == CF_P16F_D)
		putByte(0x09); // operation number
	else
		putByte(0x03); // operation number
	putByte(0x00);   // number of bytes remaining
	getByte();       // return result - no check for its value
	return 0;
}

int p16d_set_pointer(unsigned int addr) {
	if (verbose > 2)
		flsprintf(stdout, "\nSetting PC\n");
	putByte(0x0c); // operation number
	putByte(0x02); // number of bytes remaining
	putByte(addr & 0xff);
	putByte((addr >> 8) & 0xff);
	getByte(); // return result - no check for its value
	return 0;
}

int p16a_mass_erase(void) {
	if (verbose > 2)
		flsprintf(stdout, "Mass erase\n");
	putByte(0x07);
	putByte(0x00);
	getByte();
	return 0;
}

int p16a_load_config(void) {
	if (verbose > 2)
		flsprintf(stdout, "Load config\n");
	putByte(0x04);
	putByte(0x00);
	getByte();
	return 0;
}

int p16a_inc_pointer(unsigned char num) {
	if (verbose > 2)
		flsprintf(stdout, "Inc pointer %d\n", num);
	putByte(0x05);
	putByte(0x01);
	putByte(num);
	getByte();
	return 0;
}

int p16a_program_page(unsigned int ptr, unsigned char num,
                       unsigned char slow) {
	if (verbose > 2)
		flsprintf(stdout, "Programming page of %d bytes at 0x%4.4x\n", num, ptr);
	putByte(0x08);
	putByte(num + 2);
	putByte(num);
	putByte(slow);

	// CHANGE THIS LINE: Use progmem instead of file_image
	putBytes(&progmem[ptr], num);

	getByte();
	return 0;
}

int p16a_read_page(unsigned char *data, unsigned char num) {
	unsigned char i;
	if (verbose > 2)
		flsprintf(stdout, "Reading page of %d bytes\n", num);
	putByte(0x06);
	putByte(0x01);
	putByte(num / 2);
	getByte();
	for (i = 0; i < num; i++)
		*data++ = getByte();
	return 0;
}

int p16a_write_eeprom(unsigned char data) {
	putByte(0x0E);           // Command 0x0E in firmware
	putByte(0x01);           // 1 byte of data follows
	putByte(data);           // The actual EEPROM byte
	if (getByte() != 0x8E) { // Wait for ACK from firmware
		return 0;
	}
	return 1;
}

int p16a_read_eeprom(unsigned char *data, unsigned char num) {
	unsigned char i;
	if ((chip_family == CF_P16F_A) || (chip_family == CF_P16F_D)) {
		if (verbose > 2)
			flsprintf(stdout, "Reading EEPROM of %d bytes\n", num);
		if (chip_family == CF_P16F_A)
			putByte(0x0a);
		if (chip_family == CF_P16F_D)
			putByte(0x0d);
		putByte(0x01);
		putByte(num);
		getByte(); // response
		for (i = 0; i < num; i++)
			*data++ = getByte();
		return 1;
	} else
		return 0;
}

int p16a_get_devid(void) {
	unsigned char tdat[20], devid_lo, devid_hi;
	unsigned int retval;
	p16a_rst_pointer();
	p16a_load_config();
	p16a_inc_pointer(6);
	p16a_read_page(tdat, 4);
	devid_hi = tdat[(2 * 0) + 1];
	devid_lo = tdat[(2 * 0) + 0];
	if (verbose > 2)
		flsprintf(stdout, "Getting devid - lo:%2.2x,hi:%2.2x\n", devid_lo,
		          devid_hi);
	retval =
	    (((unsigned int)(devid_lo)) << 0) + (((unsigned int)(devid_hi)) << 8);
	retval = retval & devid_mask;
	return retval;
}

int p16a_get_config(unsigned char n) {
	unsigned char tdat[20], devid_lo, devid_hi;
	unsigned int retval;
	p16a_rst_pointer();
	p16a_load_config();
	p16a_inc_pointer(n);
	p16a_read_page(tdat, 4);
	devid_hi = tdat[(2 * 0) + 1];
	devid_lo = tdat[(2 * 0) + 0];
	retval =
	    (((unsigned int)(devid_lo)) << 0) + (((unsigned int)(devid_hi)) << 8);
	if (verbose > 2)
		flsprintf(stdout, "Getting config +%d - lo:%2.2x,hi:%2.2x = %4.4x\n", n,
		          devid_lo, devid_hi, retval);
	return retval;
}

int p16a_program_config(void) {
	p16a_rst_pointer();
	p16a_load_config();
	p16a_inc_pointer(7);
	p16a_program_page(2 * 0x8007, 2, 1);
	p16a_program_page(2 * 0x8008, 2, 1);
	if ((chip_family == CF_P16F_B) | (chip_family == CF_P16F_D))
		p16a_program_page(2 * 0x8009, 2, 1);
	if (chip_family == CF_P16F_D)
		p16a_program_page(2 * 0x800A, 2, 1);
	return 0;
}

int p18a_read_page(unsigned char *data, int address, unsigned char num) {
	unsigned char i;
	if (verbose > 2)
		flsprintf(stdout, "Reading page of %d bytes at 0x%6.6x\n", num, address);
	putByte(0x11);
	putByte(0x04);
	putByte(num / 2);
	putByte((address >> 16) & 0xFF);
	putByte((address >> 8) & 0xFF);
	putByte((address >> 0) & 0xFF);
	getByte();
	for (i = 0; i < num; i++) {
		*data++ = getByte();
	}
	return 0;
}

int p18a_mass_erase(void) {
	if (verbose > 2)
		flsprintf(stdout, "Mass erase\n");
	putByte(0x13);
	putByte(0x00);
	getByte();
	return 0;
}

int p18b_mass_erase(void) {
	if (verbose > 2)
		flsprintf(stdout, "Mass erase\n");
	putByte(0x23);
	putByte(0x00);
	getByte();
	return 0;
}

int p18d_mass_erase_part(unsigned long data) {
	if (verbose > 2)
		flsprintf(stdout, "Mass erase part of 0x%6.6x\n", data);
	putByte(0x30);
	putByte(0x03);
	putByte((data >> 16) & 0xFF);
	putByte((data >> 8) & 0xFF);
	putByte((data >> 0) & 0xFF);
	getByte();
	return 0;
}

int p18d_mass_erase(void) {
	if (verbose > 2)
		flsprintf(stdout, "Mass erase\n");
	p18d_mass_erase_part(0x800104);
	p18d_mass_erase_part(0x800204);
	p18d_mass_erase_part(0x800404);
	p18d_mass_erase_part(0x800804);
	/*
	p18d_mass_erase_part(0x801004);
	p18d_mass_erase_part(0x802004);
	p18d_mass_erase_part(0x804004);
	p18d_mass_erase_part(0x808004);
	*/
	p18d_mass_erase_part(0x800004);
	p18d_mass_erase_part(0x800005);
	p18d_mass_erase_part(0x800002);
	return 0;
}

int p18e_mass_erase(void) {
	if (verbose > 2)
		flsprintf(stdout, "Mass erase\n");
	p18d_mass_erase_part(0x800104);
	p18d_mass_erase_part(0x800204);
	p18d_mass_erase_part(0x800404);
	p18d_mass_erase_part(0x800804);
	p18d_mass_erase_part(0x801004);
	p18d_mass_erase_part(0x802004);
	p18d_mass_erase_part(0x804004);
	p18d_mass_erase_part(0x808004);
	p18d_mass_erase_part(0x800004);
	p18d_mass_erase_part(0x800005);
	p18d_mass_erase_part(0x800002);
	return 0;
}

int p18a_write_page(unsigned char *data, int address, unsigned char num) {
	unsigned char i, empty;
	empty = 0;
	for (i = 0; i < num; i++) {
		if (data[i] != 0xFF)
			empty = 0;
	}
	if (empty == 1) {
		if (verbose > 3)
			flsprintf(stdout, "~");
		return 0;
	}
	if (verbose > 2)
		flsprintf(stdout, "Writing A page of %d bytes at 0x%6.6x\n", num, address);
	putByte(0x12);
	putByte(4 + num);
	putByte(num);
	putByte((address >> 16) & 0xFF);
	putByte((address >> 8) & 0xFF);
	putByte((address >> 0) & 0xFF);
	for (i = 0; i < num; i++)
		putByte(data[i]);
	getByte();
	return 0;
}

int p18d_write_page(unsigned char *data, int address, unsigned char num) {
	unsigned char i, empty;
	empty = 0;
	for (i = 0; i < num; i++) {
		if (data[i] != 0xFF)
			empty = 0;
	}
	if (empty == 1) {
		if (verbose > 3)
			flsprintf(stdout, "~");
		return 0;
	}
	if (verbose > 2)
		flsprintf(stdout, "Writing D page of %d bytes at 0x%6.6x\n", num, address);
	putByte(0x31);
	putByte(4 + num);
	putByte(num);
	putByte((address >> 16) & 0xFF);
	putByte((address >> 8) & 0xFF);
	putByte((address >> 0) & 0xFF);
	for (i = 0; i < num; i++)
		putByte(data[i]);
	getByte();
	return 0;
}

int p18a_write_cfg(unsigned char data1, unsigned char data2, int address) {
	if (verbose > 2)
		flsprintf(stdout, "Writing cfg 0x%2.2x 0x%2.2x at 0x%6.6x\n", data1, data2,
		          address);
	putByte(0x14);
	putByte(6);
	putByte(0);
	putByte((address >> 16) & 0xFF);
	putByte((address >> 8) & 0xFF);
	putByte((address >> 0) & 0xFF);
	putByte(data1);
	putByte(data2);
	getByte();
	return 0;
}

int p18d_write_cfg(unsigned char data1, unsigned char data2, int address) {
	if (verbose > 2)
		flsprintf(stdout, "Writing cfg 0x%2.2x 0x%2.2x at 0x%6.6x\n", data1, data2,
		          address);
	putByte(0x32);
	putByte(6);
	putByte(0);
	putByte((address >> 16) & 0xFF);
	putByte((address >> 8) & 0xFF);
	putByte((address >> 0) & 0xFF);
	putByte(data1);
	putByte(data2);
	getByte();
	return 0;
}

int p16c_mass_erase(void) {
	if (verbose > 2)
		flsprintf(stdout, "Mass erase\n");
	putByte(0x43);
	putByte(0x00);
	getByte();
	return 0;
}

int p16c_read_page(unsigned char *data, int address, unsigned char num) {
	unsigned char i;
	address = address / 2;
	if (verbose > 2)
		flsprintf(stdout, "Reading page of %d bytes at 0x%6.6x\n", num, address);
	putByte(0x41);
	putByte(0x04);
	putByte(num / 2);
	putByte((address >> 16) & 0xFF);
	putByte((address >> 8) & 0xFF);
	putByte((address >> 0) & 0xFF);
	getByte();
	for (i = 0; i < num; i++) {
		*data++ = getByte();
	}
	//    for (i=0; i<num; i++) if (verbose>2) flsprintf(stdout,"%2.2x ",
	//    data[i]);

	return 0;
}

int p16c_write_page(unsigned char *data, int address, unsigned char num) {
	unsigned char i, empty;
	address = address / 2;
	empty = 1;
	for (i = 0; i < num; i = i + 2) {
		if ((data[i] != 0xFF) | (data[i + 1] != 0xFF))
			empty = 0;
	}
	if (verbose > 2)
		flsprintf(stdout, "Writing A page of %d bytes at 0x%6.6x\n", num, address);
	if (empty == 1) {
		if (verbose > 3)
			flsprintf(stdout, "~");
		return 0;
	}
	putByte(0x42);
	putByte(4 + num);
	putByte(num);
	putByte((address >> 16) & 0xFF);
	putByte((address >> 8) & 0xFF);
	putByte((address >> 0) & 0xFF);
	for (i = 0; i < num; i++)
		putByte(data[i]);
	getByte();
	return 0;
}

int p16c_get_devid(void) {
	unsigned char tdat[20], devid_lo, devid_hi;
	unsigned int retval;
	p16c_read_page(tdat, 0x8006 * 2, 4);
	devid_hi = tdat[(2 * 0) + 1];
	devid_lo = tdat[(2 * 0) + 0];
	if (verbose > 2)
		flsprintf(stdout, "Getting devid - lo:%2.2x,hi:%2.2x\n", devid_lo,
		          devid_hi);
	retval =
	    (((unsigned int)(devid_lo)) << 0) + (((unsigned int)(devid_hi)) << 8);
	retval = retval & devid_mask;
	return retval;
}

int p16c_write_single_cfg(unsigned char data1, unsigned char data2,
                          int address) {
	if (verbose > 2)
		flsprintf(stdout, "Writing cfg 0x%2.2x 0x%2.2x at 0x%6.6x\n", data1, data2,
		          address);
	putByte(0x44);
	putByte(6);
	putByte(0);
	putByte((address >> 16) & 0xFF);
	putByte((address >> 8) & 0xFF);
	putByte((address >> 0) & 0xFF);
	putByte(data1);
	putByte(data2);
	getByte();
	return 0;
}

int p18q_write_single_cfg(unsigned char data1, unsigned char data2,
                          int address) {
	if (verbose > 2)
		flsprintf(stdout, "Writing cfg 0x%2.2x 0x%2.2x at 0x%6.6x\n", data1, data2,
		          address);
	putByte(0x45);
	putByte(6);
	putByte(0);
	putByte((address >> 16) & 0xFF);
	putByte((address >> 8) & 0xFF);
	putByte((address >> 0) & 0xFF);
	putByte(data1);
	putByte(data2);
	getByte();
	return 0;
}

int p18q_write_page(unsigned char *data, int address, unsigned char num) {
	unsigned char i, empty;
	address = address / 2;
	empty = 1;
	for (i = 0; i < num; i = i + 2) {
		if ((data[i] != 0xFF) | (data[i + 1] != 0xFF))
			empty = 0;
	}
	if (verbose > 2)
		flsprintf(stdout, "Writing A page of %d bytes at 0x%6.6x\n", num, address);
	if (empty == 1) {
		if (verbose > 3)
			flsprintf(stdout, "~");
		return 0;
	}
	putByte(0x46);
	putByte(4 + num);
	putByte(num);
	putByte((address >> 16) & 0xFF);
	putByte((address >> 8) & 0xFF);
	putByte((address >> 0) & 0xFF);
	for (i = 0; i < num; i++)
		putByte(data[i]);
	getByte();
	return 0;
}

int p16c_write_cfg(void) {
	p16c_write_single_cfg(config_bytes[1], config_bytes[0], 0x8007);
	p16c_write_single_cfg(config_bytes[3], config_bytes[2], 0x8008);
	p16c_write_single_cfg(config_bytes[5], config_bytes[4], 0x8009);
	p16c_write_single_cfg(config_bytes[7], config_bytes[6], 0x800A);
	p16c_write_single_cfg(config_bytes[9], config_bytes[8], 0x800B);
	return 0;
}

int prog_enter_progmode(void) {
	if (verbose > 2)
		flsprintf(stdout, "Entering programming mode\n");
	if (chip_family == CF_P16F_A)
		putByte(0x01);
	else if (chip_family == CF_P16F_B)
		putByte(0x01);
	else if (chip_family == CF_P16F_D)
		putByte(0x01);
	else if (chip_family == CF_P18F_A)
		putByte(0x10);
	else if (chip_family == CF_P18F_B)
		putByte(0x10);
	else if (chip_family == CF_P18F_D)
		putByte(0x10);
	else if (chip_family == CF_P18F_E)
		putByte(0x10);
	else if (chip_family == CF_P16F_C)
		putByte(0x40);
	else if (chip_family == CF_P18F_F)
		putByte(0x40);
	else if (chip_family == CF_P18F_Q)
		putByte(0x40);
	putByte(0x00);
	getByte();
	return 0;
}

int prog_exit_progmode(void) {
	if (verbose > 2)
		flsprintf(stdout, "Exiting programming mode\n");
	putByte(0x02);
	putByte(0x00);
	getByte();
	return 0;
}

int prog_get_device_id(void) {
	unsigned char mem_str[10];
	unsigned int devid;
	if (verbose > 2)
		flsprintf(stdout, "getting ID for family %d\n", chip_family);
	if ((chip_family == CF_P16F_A) | (chip_family == CF_P16F_B) |
	        (chip_family == CF_P16F_D))
		return p16a_get_devid();
	if ((chip_family == CF_P16F_C))
		return p16c_get_devid();
	else if ((chip_family == CF_P18F_A) | (chip_family == CF_P18F_B) |
	         (chip_family == CF_P18F_D) | (chip_family == CF_P18F_E)) {
		p18a_read_page((unsigned char *)&mem_str, 0x3FFFFE, 2);
		devid = (((unsigned int)(mem_str[1])) << 8) +
		        (((unsigned int)(mem_str[0])) << 0);
		devid = devid & devid_mask;
		return devid;
	}
	if ((chip_family == CF_P18F_F) | (chip_family == CF_P18F_Q)) {
		p16c_read_page(mem_str, 0x3FFFFE * 2, 2);
		devid = (((unsigned int)(mem_str[1])) << 8) +
		        (((unsigned int)(mem_str[0])) << 0);
		devid = devid & devid_mask;
		return devid;
	}

	return 0;
}

int parse_hex(char *filename, unsigned char *buffer) {
    FILE *sf = fopen(filename, "rb");
    if (!sf) return -1;
    char *line = NULL; size_t len = 0; int read, offset = 0;
    while ((read = getlinex(&line, &len, sf)) != -1) {
        if (line[0] != ':') continue;
        int l_len, l_addr, l_type;
        sscanf(line + 1, "%2X%4X%2X", &l_len, &l_addr, &l_type);
        int eff_addr = l_addr + (offset << 16);
        if (l_type == 0) {
            for (int i = 0; i < l_len; i++) {
                int val; sscanf(line + 9 + (i * 2), "%2X", &val);
                if (eff_addr + i < PROGMEM_LEN) {
                    unsigned char b = (unsigned char)val;
                    // Mask high byte for Flash (not for EEPROM/Config area)
                    if (eff_addr + i < 0x10000 && (eff_addr + i) % 2 != 0) b &= 0x3F;
                    buffer[eff_addr + i] = b;
                }
            }
        } else if (l_type == 4) sscanf(line + 9, "%4X", &offset);
    }
    fclose(sf); return 0;
}

int main(int argc, char *argv[]) {
	int i, j, pages_performed, config, econfig, hex_ok;
	unsigned char *pm_point, *cm_point;
	unsigned char tdat[200];
	parseArgs(argc, argv);
	if (verbose > 0)
		printf("ATU PIC16F1938 programmer, version %s\n", PP_VER);
	if (verbose > 1)
		printf("Opening serial port\n");
	//initSerialPort();
	if (sleep_time > 0) {
		if (verbose > 0)
			printf("Sleeping for %d ms while arduino bootloader expires\n",
			       sleep_time);
		fflush(stdout);
		sleep_ms(sleep_time);
	}
	// INSERT THE NEW CODE HERE:
	if (hv_programming) {
		if (verbose > 0)
			printf("Switching Arduino to High Voltage Mode...\n");
		putByte(0x0B);                 // The new command ID we added to ppr.ino
		putByte(0x01);                 // Telling Arduino 1 byte of data is coming
		putByte(0x01);                 // The data byte: 1 for HVP (0 would be LVP)
		unsigned char ack = getByte(); // Wait for the 0x8B response we defined
		if (ack != 0x8B) {
			fprintf(stderr,
			        "Error: Arduino did not acknowledge HVP mode (Recv: 0x%02X)\n",
			        ack);
			exit(1);
		}
	}

	for (i = 0; i < PROGMEM_LEN; i++)
		progmem[i] = 0xFF; // assume erased memories (0xFF)
	for (i = 0; i < CONFIG_LEN; i++)
		config_bytes[i] = 0xFF;

	char *filename = argv[argc - 1];
	pm_point = (unsigned char *)(&progmem);
	cm_point = (unsigned char *)(&config_bytes);
	hex_ok =  parse_hex(filename, pm_point); // parse and write content of hex file into buffers		  
	
       // Parse the HEX file into the unified progmem buffer
    hex_ok = parse_hex(filename, pm_point); 

    if (hex_ok == 0) {
        FILE *log = fopen("buffer_map.log", "w");
        printf("\n--- Unified Buffer Mapping Report ---\n");
        if (log) fprintf(log, "Buffer Mapping for: %s\n\n", filename);

        int in_block = 0;
        int start_addr = 0;

        for (int i = 0; i < PROGMEM_LEN; i++) {
            if (progmem[i] != 0xFF) {
                if (!in_block) {
                    start_addr = i;
                    in_block = 1;
                }
            } else {
                if (in_block) {
                    int end_addr = i - 1;
                    char *region = "Unknown/Flash";
                    
                    // Logic to identify memory regions
                    if (start_addr < 0x10000) {
                        region = "FLASH MEMORY";
                    } else if (start_addr >= 0x1000E && start_addr <= 0x10015) {
                        region = "CONFIGURATION BITS";
                    } else if (start_addr >= 0x1E000) {
                        region = "EEPROM DATA";
                    }

                    // Print to Console
                    printf("[%s]\n", region);
                    printf("  Buffer Offset: 0x%05X to 0x%05X\n", start_addr, end_addr);
                    printf("  PIC Word Addr: 0x%04X\n", start_addr / 2);
                    
                    // Save to Log File
                    if (log) {
                        fprintf(log, "[%s]\n", region);
                        fprintf(log, "  Offset: 0x%05X - 0x%05X (Len: %d bytes)\n", 
                                start_addr, end_addr, (end_addr - start_addr) + 1);
                        fprintf(log, "  Data Snippet: ");
                        for(int k = start_addr; k <= end_addr && k < start_addr + 8; k++)
                            fprintf(log, "%02X ", progmem[k]);
                        fprintf(log, "\n\n");
                    }
                    
                    printf("---------------------------------------\n");
                    in_block = 0;
                }
            }
        }
        if (log) {
            printf("Detailed map saved to: buffer_map.log\n");
            fclose(log);
        }
        printf("--- End of Report ---\n\n");
    }


       return 0;

	prog_enter_progmode(); // enter programming mode and probe the target

	i = prog_get_device_id();
	if (i == devid_expected) {
		if (verbose > 0)
			printf("Device ID: %4.4x \n", i);
	} else {
		printf("Wrong device ID: %4.4x, expected: %4.4x\n", i, devid_expected);
		printf("Check for connection to target MCU, exiting now\n");
		prog_exit_progmode();
		return 1;
	}

	if (readflash == 1) {
		program = 0;
		verify = 0;
	}

	if ((hex_ok == -1) && (program == 1 && verify == 1)) {
		printf("No or wrong input HEX file.\n");
		prog_exit_progmode();
		return 1;
	}

	if (program == 1) {
        printf("Erasing & Programming..."); fflush(stdout);
        p16a_mass_erase();
        p16a_rst_pointer();
        for (int i = 0; i < 2 * flash_size; i += page_size) {
            p16a_program_page(i, page_size, 0);
            if (verbose > 1 && (i % 1024 == 0)) printf("."); fflush(stdout);
        }
        // Program Config (PIC16 Config starts at 0x8007, which is 0x1000E in HEX)
        p16a_rst_pointer();
        putByte(0x04); getByte(); // Load Config
        p16a_program_page(2 * 0x8007, 2, 1); // Config 1
        p16a_program_page(2 * 0x8008, 2, 1); // Config 2
        if (chip_family == CF_P16F_D) {
            p16a_program_page(2 * 0x8009, 2, 1); // Config 3
            p16a_program_page(2 * 0x800A, 2, 1); // Config 4
        }
	// 3. Program EEPROM
	int eeprom_base = 2 * 0xF000; // Byte offset in progmem
	printf("Programming EEPROM...");
	
	// Note: Ensure your firmware or a preceding command sets the pointer 
	// to the EEPROM start address (0xF000) before this loop.
	for (int i = 0; i < 256; i++) {
	    if (progmem[eeprom_base + i] != 0xFF) {
	        p16a_write_eeprom(progmem[eeprom_base + i]); // Now matches your 1-argument requirement
	    } else {
	        // If skipping 0xFF, you may need to manually increment the PIC pointer 
	        // to keep it in sync with 'i' if your firmware auto-increments on write.
	        p16a_inc_pointer(1); 
	    }
	}
	printf(" Done.\n");
		
	}
		
	if (verify == 1) {
        printf("Verifying..."); fflush(stdout);
        unsigned char tdat[200];
        p16a_rst_pointer();
        for (int i = 0; i < 2 * flash_size; i += page_size) {
            p16a_read_page(tdat, page_size);
            for (int j = 0; j < page_size; j++) {
                if (progmem[i + j] != tdat[j]) {
                    printf("\nFlash Error at %04X (E:%02X R:%02X)\n", i+j, progmem[i+j], tdat[j]);
                    prog_exit_progmode(); return 1;
                }
            }
            if (verbose > 1 && (i % 1024 == 0)) printf("."); fflush(stdout);
        }

        // --- FIXED CONFIG VERIFICATION ---
        int configs_to_check = (chip_family == CF_P16F_D) ? 4 : 2;
        for (int c = 0; c < configs_to_check; c++) {
            int addr = 0x8007 + c;
            int actual = p16a_get_config(7 + c);
            int expected = progmem[2 * addr] | (progmem[2 * addr + 1] << 8);
            
            if (actual != expected) {
                printf("\nConfig %d Error at %04X (E:%04X R:%04X)\n", c+1, addr, expected, actual);
                prog_exit_progmode(); return 1;
            }
            if (verbose > 1) printf("(C%d:OK)", c+1);
        }
 		
		// Verify EEPROM
        unsigned char edat[256];
        p16a_read_eeprom(edat, 256);
        for (int i = 0; i < 256; i++) {
            if (progmem[2 * 0xF000 + i] != edat[i]) {
                printf("\nEEPROM Error at %04X (E:%02X R:%02X)\n", 0xF000+i, progmem[2*0xF000+i], edat[i]);
                return 1;
            }
        }
		printf(" OK\n");
    }


	if (readflash == 1) {
		if (verbose > 0)
			printf("Reading device to %s...\n", hex_out_name);
		FILE *hf = fopen(hex_out_name, "w");
		if (!hf) {
			printf("Error opening %s\n", hex_out_name);
			exit(1);
		}

		// 1. Read FLASH
		if (verbose > 0)
			printf("Reading FLASH: ");
		if ((chip_family == CF_P16F_A) || (chip_family == CF_P16F_B) ||
				(chip_family == CF_P16F_D))
			p16a_rst_pointer();

		for (i = 0; i < flash_size * 2; i = i + page_size) {
			if ((chip_family == CF_P16F_A) || (chip_family == CF_P16F_B) ||
					(chip_family == CF_P16F_D))
				p16a_read_page(tdat, page_size);
			if (chip_family == CF_P16F_C)
				p16c_read_page(tdat, i, page_size);

			for (j = 0; j < page_size; j += 16)
				write_hex_line(hf, 16, i + j, 0, &tdat[j]);

			if (verbose > 1) {
				printf(".");
				fflush(stdout);
			}
		}

		// 2. Read EEPROM
		// PIC16 EEPROM is at word 0xF000 -> Intel Hex Byte 0x1E000 (Segment
		// 0x0001, Offset 0xE000)
		if (verbose > 0)
			printf("\nReading EEPROM: ");
		unsigned char addr_ext[2] = {0x00, 0x01};
		write_hex_line(hf, 2, 0, 4,
					   addr_ext); // Set Extended Linear Address to 0x0001

		if (chip_family == CF_P16F_D)
			p16d_set_pointer(0xf000);

		for (i = 0; i < 256; i = i + page_size) {
			if ((chip_family == CF_P16F_A) || (chip_family == CF_P16F_D))
				p16a_read_eeprom(tdat, page_size);

			for (j = 0; j < page_size; j += 16)
				write_hex_line(hf, 16, 0xE000 + i + j, 0, &tdat[j]);

			if (verbose > 1) {
				printf(".");
				fflush(stdout);
			}
		}

		// 3. Read Config NEED CHECKING! MAY BE FAILING IN DUMPS
		if (verbose > 0)
			printf("\nReading Config: ");
			unsigned char cdat[16];
			memset(cdat, 0xFF, 16);

		if ((chip_family == CF_P16F_A) || (chip_family == CF_P16F_B) ||
				(chip_family == CF_P16F_D)) {
			int num_configs = (chip_family == CF_P16F_D) ? 4 : 2;
			for (int c = 0; c < num_configs; c++) {
				int val = p16a_get_config(7 + c);
				cdat[(7 + c) * 2 - 16] = val & 0xFF;
				cdat[(7 + c) * 2 - 16 + 1] = (val >> 8) & 0xFF;
			}
		} else if (chip_family == CF_P16F_C) {
			p16c_read_page(cdat, 0x8007 * 2, 10);
		}
		write_hex_line(hf, 16, 0, 0, cdat);

		// 4. End of File
		fprintf(hf, ":00000001FF\n");
		fclose(hf);
		if (verbose > 0)
			printf("\nDone.\n");
	}
		prog_exit_progmode();
		return 0;
}


/*
 * emucd.c - userspace daemon for Innodisk EMUC-B201 CAN interface driver
 *
 * Copyright (c) 2016 Innodisk Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Send feedback to <linux-can@vger.kernel.org>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <syslog.h>
#include <errno.h>
#include <pwd.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <termios.h>
#include <linux/tty.h>
#include <linux/sockios.h>
#include <linux/version.h>

#include "lib_emuc.h"

/*
 * Ldisc number for emuc.
 * Beform 3.1.0, the ldisc number is private define
 * in kernel, usrspace application cannot use it.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
#define N_EMUC (NR_LDISCS - 1)
#endif

/* Change this to whatever your daemon is called */
#define DAEMON_NAME "emucd"

/* The length of ttypath buffer */
#define TTYPATH_LENGTH	64

static void print_version(char *prg)
{
	fprintf(stdout, "%s 1.2\n\n", prg);
	exit(EXIT_SUCCESS);
}

static void print_usage(char *prg)
{
	fprintf(stderr, "\nUsage: %s [options] <tty> [canif-name] [canif2-name]\n\n", prg);
	fprintf(stderr, "Options: -s <speed>[<speed>] (set CAN speed 3..7)\n");
	fprintf(stderr, "                3: 50 KBPS\n");
	fprintf(stderr, "                4: 125 KBPS\n");
	fprintf(stderr, "                5: 250 KBPS\n");
	fprintf(stderr, "                6: 500 KBPS\n");
	fprintf(stderr, "                7: 1 MBPS\n");
	fprintf(stderr, "         -F         (stay in foreground; no daemonize)\n");
	fprintf(stderr, "         -h         (show this help page)\n");
	fprintf(stderr, "         -v         (show version info)\n");
	fprintf(stderr, "\nExamples:\n");
	fprintf(stderr, "emucd -s5 ttyACM0\n");
	fprintf(stderr, "emucd -s65 /dev/ttyACM0 can0 can1\n");
	fprintf(stderr, "\n");
	exit(EXIT_FAILURE);
}

static int emucd_running;
static int exit_code;
static char ttypath[TTYPATH_LENGTH];

static void child_handler(int signum)
{
	switch (signum) {

	case SIGUSR1:
		/* exit parent */
		exit(EXIT_SUCCESS);
		break;
	case SIGALRM:
	case SIGCHLD:
		syslog(LOG_NOTICE, "received signal %i on %s", signum, ttypath);
		exit_code = EXIT_FAILURE;
		emucd_running = 0;
		break;
	case SIGINT:
	case SIGTERM:
		syslog(LOG_NOTICE, "received signal %i on %s", signum, ttypath);
		exit_code = EXIT_SUCCESS;
		emucd_running = 0;
		break;
	}
}

/* RS232 Lib included in EMUC library */
extern char comports[38][16];

static int look_up_emuc_comport(const char *tty)
{
	int i;
	for (i = 0; i < 38; i++) {
		if (strcmp(tty, comports[i]) == 0)
			return i;
	}
	return -1; // invalid port number for EMUC library
}

static int check_can_speed_format(const char * speed)
{
	int len = strlen(speed);
	const char *p;

	if (len < 1 || len > 2)
		return -1;

	for (p = speed; *p == '\0'; p++) {
		if (*p < '3' || *p > '7')
			return -1;
	}

	return 0;
}

static const char * look_up_can_speed(int speed)
{
	switch (speed) {
		case 0: return "5 KBPS";
		case 1: return "10 KBPS";
		case 2: return "20 KBPS";
		case 3: return "50 KBPS";
		case 4: return "125 KBPS";
		case 5: return "250 KBPS";
		case 6: return "500 KBPS";
		case 7: return "1 MBPS";
		default: return "unknown";
	}
}

int main(int argc, char *argv[])
{
	char *tty = NULL;
	char const *devprefix = "/dev/";
	char *name[2];
	char buf[IFNAMSIZ+1];
	struct termios tios;
	speed_t old_ispeed;
	speed_t old_ospeed;

	int opt;
	char *speed = NULL;
	int run_as_daemon = 1;
	char *pch;
	int ldisc;
	int fd;
	int channel;

#ifdef N_EMUC
	ldisc = N_EMUC;
#else
	ldisc = 17; // N_SLCAN
#endif

	name[0] = NULL;
	name[1] = NULL;
	ttypath[0] = '\0';

	while ((opt = getopt(argc, argv, "s:v?hF")) != -1) {
		switch (opt) {
		case 's':
			speed = optarg;
			if (check_can_speed_format(speed) < 0)
				print_usage(argv[0]);
			break;
		case 'F':
			run_as_daemon = 0;
			break;
		case 'v':
			print_version(argv[0]);
			break;
		case 'h':
		case '?':
		default:
			print_usage(argv[0]);
			break;
		}
	}

	/* Initialize the logging interface */
	openlog(DAEMON_NAME, LOG_PID, LOG_LOCAL5);

	/* Parse serial device name and optional can interface name */
	tty = argv[optind];
	if (NULL == tty)
		print_usage(argv[0]);

	name[0] = argv[optind + 1];
	if (name[0])
		name[1] = argv[optind + 2];

	/* Prepare the tty device name string */
	pch = strstr(tty, devprefix);
	if (pch != tty)
		snprintf(ttypath, TTYPATH_LENGTH, "%s%s", devprefix, tty);
	else
		snprintf(ttypath, TTYPATH_LENGTH, "%s", tty);

	syslog(LOG_INFO, "starting on TTY device %s", ttypath);

	/* Set can spped by EMUC library */
	if (speed) {
		int port = look_up_emuc_comport(ttypath);
		if (port < 0) {
			syslog(LOG_ERR, "unknown comport");
			exit(EXIT_FAILURE);
		}
		if (!EMUCOpenDevice(port)) {
			if (strlen(speed) == 2) {
				int sp = atoi(speed);
				syslog(LOG_INFO, "Set can speed to %s on channel 1", look_up_can_speed(sp / 10));
				EMUCSetCAN(port, EMUC_CH_1, (sp / 10));
				syslog(LOG_INFO, "Set can speed to %s on channel 2", look_up_can_speed(sp % 10));
				EMUCSetCAN(port, EMUC_CH_2, (sp % 10));
			} else {
				syslog(LOG_INFO, "set can speed to %s on both channel", look_up_can_speed(atoi(speed)));
				EMUCSetCAN(port, EMUC_CH_BOTH, atoi(speed));
			}
			EMUCCloseDevice(port);
		} else {
			syslog(LOG_ERR, "fail to open comport");
			exit(EXIT_FAILURE);
		}
	}

	/* Daemonize */
	if (run_as_daemon) {
		if (daemon(0, 0)) {
			syslog(LOG_ERR, "failed to daemonize");
			exit(EXIT_FAILURE);
		}
	}
	else {
		/* Trap signals that we expect to receive */
		signal(SIGINT, child_handler);
		signal(SIGTERM, child_handler);
	}

	/* */
	emucd_running = 1;

	/* Now we are a daemon -- do the work for which we were paid */
	fd = open(ttypath, O_RDWR | O_NONBLOCK | O_NOCTTY);
	if (fd < 0) {
		syslog(LOG_NOTICE, "failed to open TTY device %s\n", ttypath);
		perror(ttypath);
		exit(EXIT_FAILURE);
	}

	/* Configure baud rate */
	memset(&tios, 0, sizeof(struct termios));
	if (tcgetattr(fd, &tios) < 0) {
		syslog(LOG_NOTICE, "failed to get attributes for TTY device %s: %s\n", ttypath, strerror(errno));
		exit(EXIT_FAILURE);
	}

	/* Get old values for later restore */
	old_ispeed = cfgetispeed(&tios);
	old_ospeed = cfgetospeed(&tios);

	/* Reset UART settings */
	cfmakeraw(&tios);
	tios.c_iflag &= ~IXOFF;
	tios.c_cflag &= ~CRTSCTS;

	/* Baud Rate */
	cfsetispeed(&tios, B9600);
	cfsetospeed(&tios, B9600);

	/* apply changes */
	if (tcsetattr(fd, TCSADRAIN, &tios) < 0)
		syslog(LOG_NOTICE, "Cannot set attributes for device \"%s\": %s!\n", ttypath, strerror(errno));

	/* set slcan like discipline on given tty */
	if (ioctl(fd, TIOCSETD, &ldisc) < 0) {
		perror("ioctl TIOCSETD");
		exit(EXIT_FAILURE);
	}
	
	for (channel = 0; channel < 2; channel++) {

		/* retrieve the name of the created CAN netdevice */
		if (ioctl(fd, SIOCGIFNAME, buf) < 0) {
			perror("ioctl SIOCGIFNAME");
			exit(EXIT_FAILURE);
		}

		syslog(LOG_NOTICE, "attached TTY %s channel %d to netdevice %s\n", ttypath, channel, buf);

		/* try to rename the created netdevice */
		if (name[channel]) {
			struct ifreq ifr;
			int s = socket(PF_INET, SOCK_DGRAM, 0);

			if (s < 0)
				perror("socket for interface rename");
			else {
				strncpy(ifr.ifr_name, buf, IFNAMSIZ);
				strncpy(ifr.ifr_newname, name[channel], IFNAMSIZ);

				if (ioctl(s, SIOCSIFNAME, &ifr) < 0) {
					syslog(LOG_NOTICE, "netdevice %s rename to %s failed\n", buf, name[channel]);
					perror("ioctl SIOCSIFNAME rename");
					exit(EXIT_FAILURE);
				} else
					syslog(LOG_NOTICE, "netdevice %s renamed to %s\n", buf, name[channel]);

				close(s);
			}
		}
	}

	/* The Big Loop */
	while (emucd_running)
		sleep(1); /* wait 1 second */

	/* Reset line discipline */
	syslog(LOG_INFO, "stopping on TTY device %s", ttypath);
	ldisc = N_TTY;
	if (ioctl(fd, TIOCSETD, &ldisc) < 0) {
		perror("ioctl TIOCSETD");
		exit(EXIT_FAILURE);
	}

	/* Reset old rates */
	cfsetispeed(&tios, old_ispeed);
	cfsetospeed(&tios, old_ospeed);

	/* apply changes */
	if (tcsetattr(fd, TCSADRAIN, &tios) < 0)
		syslog(LOG_NOTICE, "Cannot set attributes for device \"%s\": %s!\n", ttypath, strerror(errno));

	/* Finish up */
	syslog(LOG_NOTICE, "terminated on %s", ttypath);
	closelog();
	return exit_code;
}

/* vim: set noexpandtab ts=8 sw=8: */

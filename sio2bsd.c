# define VERSION "1"
# define REVISION "19"

/* SIO2BSD, (c) 2005-2012 KMK <drac030@krap.pl>
 *
 * CHANGES:
 *
 * rev. 19:
 * - added an option for additional delay to support communication via 
 *   Bluetooth
 *
 * rev. 18:
 * - better synchronization after a reset at Atari's side
 * - no long timeout at first status command in high baudrates (use -m)
 * - added an option to block PERCOM commands (810/1050 emulation)
 *
 * rev. 17:
 * - FREAD/FFIRST/FNEXT will return status $03 when current operation has
 *   returned the last byte of a file and the next one is about to return
 *   136 only (this is what DOS 2.0 does).
 *
 * rev. 16:
 * - fixed bug in PCLink GETCWD: unterminated string
 * - fixed bug in PCLink REMOVE: the fatr1 byte sent by the client should
 *   be ignored
 * 
 * rev. 15:
 * - allow custom baudrates on FreeBSD as well
 * - PCLink: if a file appearing in PCL: directory was in fact a symlink,
 *   opening this file for writing resulted in replacing the symlink with
 *   a newly created regular file.
 * - fixed inconsistency in timestamp2mtime(), there was a blurp of 
 *   unreachable code there (but causing no bad effects).
 * - -f flag is now attached to specified atr (mono)
 * - delay in format_atr() suppressed for a call from make_atr() and
 *   shortened to 1/10 of the previous value otherwise.
 * - sig() now displays the name of the signal it caught
 * - PCLink: with -u upper case directories are now accepted
 * - -c mono-draco constant (mono)
 * - bugfix to mkatr -d 32m (bad calculations of pars)
 *
 * rev. 14:
 * - in SIO log, added info if US is enabled during command execution
 * - added experimental ULTRA modes SIOx4 and SIOx6
 * - added another delay in sio_ack() (mono)
 * - added experimental POKEY quartz setting (-q pal/ntsc/ntscf/quartz)
 *   for -i switch (mono)
 * - added full size of first 3 sectors for dd disks in mkatr (-f) (mono)
 * - refactoring of mkatr (and added 32m density preset) (mono)
 *
 * rev. 13:
 * - added explicit ATR structure zeroing in make_atr()
 * - PCLink fix: if a file was being overwritten, its timestamp didn't change
 * - added missing # include <sys/ioctl.h>
 *
 * rev. 12:
 * - SIO commands >= 0x80 will now return NAK instead of timing out
 * - If tracks == 1, WRITE PERCOM will now be acknowledged, but ignored
 * - PCLink: REMOVE will now only pick-up non-directories
 * - PCLink: FOPEN will now explicitly delete files to be replaced
 *
 * rev. 11:
 * - added a commandline option to tune the turbo baudrate without recompiling
 * - more Cygwin-related changes
 * - fixes to minimize SIO command retries
 * - applied mono's code to select arbitrary HS-indexes (Linux only)
 *
 * rev. 10:
 * - formatting a 256-byte ATR will now force first 3 sectors to be 128-byters
 * - Cygwin compatibility stuff
 * - disable PCLink command interpretation if no folders were mounted
 *
 * rev. 9:
 * - PCLink: added CHVOL (new kernel function)
 * - PCLink: GETDFREE modified accordingly
 *
 * rev. 8:
 * - PCLink: minor bugfix in MKDIR
 * - PCLink: incompatible protocol change and optimization in FNEXT
 *
 * rev. 7:
 * - PCLink: added forgotten PCLink function RENAME/RENDIR
 * - PCLink: when timestamp is not defined, use current time instead of 1-1-1970
 *
 * rev. 6:
 * - bugfixes in general SIO code
 * - added PCLink support
 *
 * rev. 5:
 * - minor changes related to delays
 * - fixed some compile-time warnings
 *
 * rev. 4:
 * - APE time protocol
 * - command line option -s fname (defining the serial device)
 * - printer handling and the option -p fname
 * - ATASCII->ASCII translation for P: device
 * - some cleanups and minor bugfixes
 *
 * rev. 3:
 * - giving up the loop calibration
 * - correcting one place related to endianess
 * - added versioning
 *
 * rev. 2:
 * - file locking to prevent opening the same disk image twice
 */

# include <math.h>		/* lround */
# include <stdio.h>
# include <ctype.h>		/* toupper */
# include <errno.h>
# include <fcntl.h>
# include <poll.h>
# include <signal.h>
# include <stdlib.h>
# include <string.h>		/* strcmp */
# include <termios.h>
# include <time.h>
# include <unistd.h>
# include <dirent.h>

# include <sys/ioctl.h>
# include <sys/resource.h>	/* setpriority */
# include <sys/stat.h>
# include <sys/time.h>
# include <sys/types.h>

# ifdef __linux__
# include <linux/serial.h>
# endif

# define SIOTRACE

# include "sio2bsd.h"

# define BASIC_DELAY 2000

# define POKEY_PAL_HZ 1773447.0
# define POKEY_NTSC_HZ 1789790.0
# define POKEY_NTSC_FREDDY_HZ 1789772.5
/* 1781618.5 - AspeQt assumes round average PAL-NTSCFREDDY value = 1781610.0 */
# define POKEY_AVG_HZ ((POKEY_NTSC_HZ+POKEY_PAL_HZ)/2)
# define POKEY_CONST 7.1861

static DEVICE device[8][16];	/* 8 devices, 16 units each */

# ifdef ULTRA
static ushort turbo_on = 0;
static ushort turbo_ix = ULTRA;	/* SIO speed multiplier (baudrate=ULTRAx19200) */
static ushort hs_ix = HSIDX;	/* HS_INDEX */
static ushort bt_delay = 1;
static double pokey_hz = POKEY_AVG_HZ;	/* POKEY quartz frequency (PAL/NTSC average by default) */
static double pokey_const = POKEY_CONST;  /* POKEY nonlinearity constant */
# endif
static SIOSPEED siospeed[8];

static uchar percom_ed[8] = { 0x28, 0x03, 0x00, 0x1a, 0x00, 0x04, 0x00, 0x80 };
static uchar percom_qd[8] = { 0x28, 0x03, 0x00, 0x12, 0x01, 0x04, 0x01, 0x00 };
static uchar percom_hd[8] = { 0x01, 0x03, 0xff, 0xfe, 0x00, 0x04, 0x01, 0x00 };
static uchar percom_hd32[8] = { 0x01, 0x03, 0xff, 0xfe, 0x00, 0x04, 0x02, 0x00 };

static const char *pcs[] = { "B7", "B6", "B5", "B4", "LARGE", "MFM", "8INCH", "RSVD" };  
static const char *pcc[] = { "", "", "", "", "SMALL", "FM", "5.25INCH", "" };

static uchar outbuf[1026];
static uchar inpbuf[1026];

static char dpath[1024];

# ifdef SIOTRACE
static int log_flag = 0;		/* enable more SIO messages, if 1 */
# endif

static int block_percom = 0;
static int use_command = 0;

static int serial_fd = -1;
static int printer_fd = -1;

static struct termios dflt;

static int pclcnt = 1;
static int drvcnt = 1;

static uid_t our_uid = 0;

/* Helpers */

static void
sio2bsd_itsme(void)
{
	printf("\nSIO2BSD " VERSION "." REVISION ", (c) " DYEAR " by KMK/DLT\n");
}

static void
sio2bsd_usage(void)
{
	sio2bsd_itsme();

	printf("\nsio2bsd [opts] [-f] drive [-f] drive ...\n");
	printf("\nWhere 'opts' are:\n");

	printf("-m        - use COMMAND line\n");
# ifdef SIOTRACE	
	printf("-l        - extended log messages\n");
# endif
	printf("-s fname  - serial device (\"" SERIAL "\" by default)\n");
# ifdef ULTRA
	printf("-b n      - set turbo to 19200*n (n<8)\n");
# endif
	printf("-d n      - additional delay required for Bluetooth communication\n");
	printf("-p fname  - printer file\n");
	printf("-t        - enable ATASCII->ASCII translation for printer\n");
# if UPPER_DIR==0
# ifndef __CYGWIN__
	printf("-u        - accept uppercase characters only in PCLink dirs\n");
# else
	printf("-u        - use uppercase characters only in PCLink dirs\n");
# endif
# else
# ifndef __CYGWIN__
	printf("-u        - accept lowercase characters only in PCLink dirs\n");
# else
	printf("-u        - use lowercase characters only in PCLink dirs\n");
# endif
# endif
	printf("-8        - block PERCOM commands\n");

	printf("\n-f drive  - first 3 sectors of new formatted DD disk have full size in ATR\n\n");
	printf("and 'drive' can be one of the following:\n\n");

	printf("ATR file  - the image file will be mounted for sector I/O\n");
	printf("directory - the directory will be mounted as PCLink drive\n");
	printf("-         - none, this drive will remain unassinged\n\n");
	
	printf("Number of drives (ATR or PCLink) is limited to 16.\n\n");
	
# ifdef ULTRA
	printf("Options enbled by -b 0 (custom turbo speed):\n");
	printf("-i n      - set HSINDEX to n\n");
	printf("-q hz     - set accurate POKEY frequency to hz\n");
	printf("            \"pal\" set %.1f Hz\n", POKEY_PAL_HZ);
	printf("            \"ntsc\" set %.1f Hz\n", POKEY_NTSC_HZ);
	printf("            \"ntscf\" set %.1f Hz as of FREDDY NTSC machines\n", POKEY_NTSC_FREDDY_HZ);
	printf("            by default average PAL/NTSC frequency (%.3f Hz) is using\n", POKEY_AVG_HZ);
	printf("-c x      - set POKEY nonlinearity constant to x (%f is being used by default)\n", POKEY_CONST);
# endif

}

static void
mkatr_usage(void)
{
	sio2bsd_itsme();
	printf("\nmkatr [opts] fname\n");
	printf("\nWhere 'opts' are:\n\n");

	printf("-d density - one of: 90k,   130k,  180k,  360k,  720k,  1440k, 16m, 32m\n");
	printf("             or:     ss/sd, ss/ed, ss/dd, ds/dd, ds/qd, ds/hd\n");
	printf("-t tracks  - number of tracks (40)\n");
	printf("-s spt     - number of sectors per track (18)\n");
	printf("-h heads   - number of heads (1)\n");
	printf("-b bps     - bytes per sector (128)\n\n");
	
	printf("-f         - first 3 sectors of DD disk have full size in ATR\n\n");
			
	printf("fname      - the ATR image file name\n\n");
}

static int
serlock(void)
{
	long r;
	struct stat sb;
	const char *tmpdir = "/tmp";
	const char *vars[4] = { "TMP", "TEMP", "HOME", NULL };

	r = stat(tmpdir, &sb);

	if ((r < 0) || !S_ISDIR(sb.st_mode))
	{
		ushort x = 0;

		while (vars[x] != NULL)
		{
			tmpdir = getenv(vars[x]);
			if (tmpdir != NULL)
				break;
			tmpdir = "";
			x++;
		};
	}

	sprintf(dpath, "%s/sio2bsd.%ld", tmpdir, (ulong)our_uid);

	(void)mkdir(dpath, S_IRUSR|S_IWUSR|S_IXUSR);

	strcat(dpath, "/");
	strcat(dpath, SERLOCK);

	if (stat(dpath, &sb) < 0)
	{
		int fd;

		fd = creat(dpath, S_IRUSR|S_IWUSR);

		if (fd < 0)
		{
			printf("Cannot create '%s', %s (%d)\n", dpath, strerror(errno), errno);

			return fd;
		}

		close(fd);

		return 0;
	}

	return -1;
}

static void
report_percom(ushort d)
{
	int i;

	if (device[3][d].percom.flags & 0x08)
	{
		printf("PERCOM: trk %d, step %d, spt %ld, bps %d, flags %02x (", \
			device[3][d].percom.trk, device[3][d].percom.step, \
			(long)device[3][d].percom.heads * 65536 + \
			(long)device[3][d].percom.spt_hi * 256 + \
			(long)device[3][d].percom.spt_lo, \
			device[3][d].percom.bps_hi * 256 + device[3][d].percom.bps_lo, \
			device[3][d].percom.flags);
	}
	else
	{
		printf("PERCOM: trk %d, step %d, spt %d, heads %d, bps %d, flags %02x (", \
			device[3][d].percom.trk, device[3][d].percom.step, \
			device[3][d].percom.spt_hi * 256 + device[3][d].percom.spt_lo, \
			device[3][d].percom.heads + 1,\
			device[3][d].percom.bps_hi * 256 + device[3][d].percom.bps_lo, \
			device[3][d].percom.flags);
	}

	for (i = 7; i >= 0; i--)
	{
		if (device[3][d].percom.flags & 1<<i)
			printf("%s-", pcs[i ^ 7]);
		else
			if (*pcc[i ^ 7])
				printf("%s-", pcc[i ^ 7]);
	}

	printf("\b)\n");
}

static int
setup_percom(ushort d, uchar *ibuf)
{
	ulong maxsec;
	ushort bps, spt;

	device[3][d].percom.trk = ibuf[0];
	device[3][d].percom.step = ibuf[1];
	device[3][d].percom.spt_hi = ibuf[2];
	device[3][d].percom.spt_lo = ibuf[3];
	device[3][d].percom.heads = ibuf[4];
	device[3][d].percom.flags = ibuf[5];
	device[3][d].percom.bps_hi = ibuf[6];
	device[3][d].percom.bps_lo = ibuf[7];

	report_percom(d);

	maxsec = device[3][d].percom.spt_hi * 256 + device[3][d].percom.spt_lo;
	maxsec *= device[3][d].percom.trk;

	bps = device[3][d].percom.bps_hi * 256 + device[3][d].percom.bps_lo;

	spt = device[3][d].percom.spt_hi * 256 + device[3][d].percom.spt_lo;

	if ((bps != 0x0080) && (bps != 0x0100) && (bps != 0x0200) && (bps != 0x0400))
		return -1;

	if (bps >= 256 && ((device[3][d].percom.flags & 0x04) == 0))
		return -1;

	if (spt > 18 && ((device[3][d].percom.flags & 0x04) == 0))
		return -1;

	if (device[3][d].percom.flags & 0x08)
		maxsec += (device[3][d].percom.heads * 65536);
	else
	{
		if (device[3][d].percom.trk == 40 || device[3][d].percom.trk == 80 || device[3][d].percom.trk == 77)
			maxsec *= (device[3][d].percom.heads + 1);
	}

	device[3][d].maxsec = maxsec;
	device[3][d].bps = bps;

	return 0;
}

/* Init various structures */

static void
device_reset(ushort devno, ushort d)
{
	bzero(&device[devno][d], sizeof(DEVICE));

	device[devno][d].status.err = 0xff;
	device[devno][d].status.tmot = 0xe0;

	device[devno][d].fd = -1;
}

static long
drive_setup(ushort d, ulong size, ushort bps)
{
	ulong sectors;

	device[3][d].bps = bps;

	/* For double density:
	 * sectors 1-3 in some ATR files are physically 128-byte wide,
	 * while in other ATR files these take 768 bytes, and the first
	 * half of this area is filled with the actual data.
	 */
	if ((size % bps) == 0)
	{
		device[3][d].full13 = 1;	/* this affects seek()s in read/write ops */
		sectors = size / bps;
	}
	else
		sectors = ((size - 384) / bps) + 3;

	if (sectors < 1)
		return -1;	/* too small */

	device[3][d].percom.step = 3;
	device[3][d].percom.bps_hi = bps / 256;
	device[3][d].percom.bps_lo = bps % 256;

	if (sectors == 720)			/* SD and DD */
	{			
		device[3][d].percom.trk = 40;
		device[3][d].percom.spt_hi = 0;
		device[3][d].percom.spt_lo = 18;
		device[3][d].percom.heads = 0;
		device[3][d].percom.flags = (bps == 128) ? 0x00 : 0x04;
	}
	else if (bps == 128 && sectors == 1040)		/* ED */
	{
		device[3][d].percom.trk = 40;
		device[3][d].percom.spt_hi = 0;
		device[3][d].percom.spt_lo = 26;
		device[3][d].percom.heads = 0;
		device[3][d].percom.flags = 0x04;	/* MFM */
	}
	else if (sectors == 1440)		/* DS/DD */
	{
		device[3][d].percom.trk = 40;
		device[3][d].percom.spt_hi = 0;
		device[3][d].percom.spt_lo = 18;
		device[3][d].percom.heads = 1;
		device[3][d].percom.flags = (bps == 128) ? 0x00 : 0x04;
	}
	else if (sectors == 2002)		/* 77 tracks, 1 side */
	{
		device[3][d].percom.trk = 77;
		device[3][d].percom.spt_hi = 0;
		device[3][d].percom.spt_lo = 26;
		device[3][d].percom.heads = 0;
		device[3][d].percom.flags = (bps == 128) ? 0x02 : 0x06;
	}
	else if (sectors == 2880)		/* 720k */
	{
		device[3][d].percom.trk = 80;
		device[3][d].percom.spt_hi = 0;
		device[3][d].percom.spt_lo = 18;
		device[3][d].percom.heads = 1;
		device[3][d].percom.flags = (bps == 128) ? 0x00 : 0x04;
	}
	else if (sectors == 4004)		/* 77 tracks, 2 sides */
	{
		device[3][d].percom.trk = 77;
		device[3][d].percom.spt_hi = 0;
		device[3][d].percom.spt_lo = 26;
		device[3][d].percom.heads = 1;
		device[3][d].percom.flags = (bps == 128) ? 0x02 : 0x06;
	}
	else if (sectors == 5760)		/* 1440k */
	{
		device[3][d].percom.trk = 80;
		device[3][d].percom.spt_hi = 0;
		device[3][d].percom.spt_lo = 36;
		device[3][d].percom.heads = 1;
		device[3][d].percom.flags = (bps == 128) ? 0x00 : 0x04;
	}
	else					/* HGW */
	{
		device[3][d].percom.trk = 1;
		device[3][d].percom.spt_hi = (sectors % 65536) / 256;
		device[3][d].percom.spt_lo = (sectors % 65536) % 256;
		device[3][d].percom.heads = sectors / 65536;

		device[3][d].percom.flags = (bps == 128) ? 0x00 : 0x04;

		if (sectors / 65536)
			device[3][d].percom.flags |= 0x08;
	}

	device[3][d].maxsec = sectors;

	return 0;
}

/* Setup status bits to reflect the current density.
 * Setting this correctly is rather critical for most DOSes.
 */
static void
setup_status(ushort d)
{
	device[3][d].status.stat &= ~0xa0;

	/* Seems that bit 5 indicates 256-byte sector,
	 * and not MFM density.
	 */
	if (device[3][d].bps >= 256)
		device[3][d].status.stat |= 0x20;

	if ((device[3][d].maxsec == 1040) && \
		(device[3][d].bps == 128) && \
			(device[3][d].percom.flags & 0x04) && \
				(device[3][d].percom.heads == 0) && \
					(device[3][d].percom.trk == 40))
	{
		device[3][d].status.stat |= 0x80;
	}
}

static void
unix_time_2_sdx(time_t *todp, uchar *ob)
{
	struct tm *t;
	uchar yy;

	bzero(ob, 6);

	if (*todp == 0)
		return;

	t = localtime(todp);

	yy = t->tm_year;
	while (yy >= 100)
		yy-=100;

	ob[0] = t->tm_mday;
	ob[1] = t->tm_mon + 1;
	ob[2] = yy;
	ob[3] = t->tm_hour;
	ob[4] = t->tm_min;
	ob[5] = t->tm_sec;
}

static void
get_sdx_time(uchar *ob)
{
	struct timeval tod, *todp = &tod;

	gettimeofday(todp, NULL);
	unix_time_2_sdx((time_t *)&todp->tv_sec, ob+1);

	ob[0] = 0xff;
}

/* ============== SIO low level ================= */

static void
wait_for_command_drop(void)
{
	int c_state, n_state, c_mask;

	if (use_command == 0)
		return;

	if (ioctl(serial_fd, TIOCMGET, &c_state) >= 0)
	{
		do
		{
			ioctl(serial_fd, TIOCMGET, &n_state);
		}
		while (c_state == n_state);

		c_mask = c_state ^ n_state;

		if (log_flag)
		{
			printf("CMD = ");

			switch (c_mask)
			{
				case TIOCM_LE:
				{
					printf("LE (Line Enable)\n");
					break;
				}
				case TIOCM_DTR:
				{
					printf("DTR (Data Terminal Ready)\n");
					break;
				}
				case TIOCM_RTS:
				{
					printf("RTS (Request To Send)\n");
					break;
				}
				case TIOCM_ST:
				{
					printf("ST (Secondary Transmit)\n");
					break;
				}
				case TIOCM_SR:
				{
					printf("SR (Secondary Receive)\n");
					break;
				}
				case TIOCM_CTS:
				{
					printf("CTS (Clear To Send)\n");
					break;
				}
# ifdef __linux__
				case TIOCM_CD:
# else
				case TIOCM_DCD:
# endif
				{
					printf("DCD (Data Carrier Detect)\n");
					break;
				}
				case TIOCM_RI:
				{
					printf("RI (Ring Indicator)\n");
					break;
				}
				case TIOCM_DSR:
				{
					printf("DSR (Data Set Ready)\n");
					break;
				}
				default:
				{
					printf("???\n");
					break;
				}
			}
		}
	}
}

/* Calculate Atari-style CRC for the given buffer
 */
static uchar
calc_checksum(uchar *buf, int how_much)
{
	uchar cksum = 0;
	ushort nck;
	int i;

	for (i = 0; i < how_much; i++)
	{
		nck = cksum + buf[i];
		cksum = (nck > 0x00ff) ? ((nck & 0x00ff) + 1) : (nck & 0x00ff);
	}

	return cksum;
}

#  define COM_COMD 0
#  define COM_DATA 1

# ifdef COMMAND_LINE
static int comstate = 0;
static int cmd_mask = 0;
static int cmd_line_valid = -1;
# endif

static void
com_read(uchar *buf, int size, const ushort type)
{
	int r, i = 0;
# ifndef COMMAND_LINE
	(void)type;
# else
	int cmd_state = 0;

	if ((type == COM_COMD) && (cmd_line_valid > 0))
	{
		int new_state;

		do
		{
			(void)ioctl(serial_fd, TIOCMGET, &new_state);
			cmd_state = new_state & cmd_mask;
		} while (cmd_state == 0);

		while (size)
		{
			r = read(serial_fd, buf+i, 1);
			if (r < 0)
			{
				printf("FATAL: %s(): %s (%d)\n", __FUNCTION__, strerror(errno), errno);
				sig(0);
			}
			i += r;
			size -= r;
		}

		return;
	}
	else
	{
		int n_state;
# endif
		while (size)
		{
			r = read(serial_fd, buf+i, 1);
			if (r < 0)
			{
				printf("FATAL: %s(): %s (%d)\n", __FUNCTION__, strerror(errno), errno);
				sig(0);
			}
# ifdef COMMAND_LINE
			if ((type == COM_COMD) && (cmd_line_valid < 0))
				if (ioctl(serial_fd, TIOCMGET, &n_state) >= 0)
					cmd_state |= n_state;
# endif
			if ((type == COM_COMD) && (i == 0) && (buf[i] == 0xff))		/* ignore $FF the OS sends at reset time */
				continue;

			i += r;
			size -= r;
		}
# ifdef COMMAND_LINE
	}

	/* at first try to determine if the COMMAND line is in use, and which one is it */
	if ((type == COM_COMD) && (cmd_line_valid < 0))
	{
		cmd_line_valid = 0;

		cmd_mask = comstate ^ cmd_state;

		if (cmd_mask == 0)
		{
			printf("COMMAND is not connected\n");
		}
		else
		{
			cmd_line_valid = 1;

			printf("COMMAND is tied to ");

			switch(cmd_mask)
			{
				case TIOCM_LE:
				{
					printf("LE (Line Enable)\n");
					break;
				}
				case TIOCM_DTR:
				{
					printf("DTR (Data Terminal Ready)\n");
					break;
				}
				case TIOCM_RTS:
				{
					printf("RTS (Request To Send)\n");
					break;
				}
				case TIOCM_ST:
				{
					printf("ST (Secondary Transmit)\n");
					break;
				}
				case TIOCM_SR:
				{
					printf("SR (Secondary Receive)\n");
					break;
				}
				case TIOCM_CTS:
				{
					printf("CTS (Clear To Send)\n");
					break;
				}
				case TIOCM_DCD:
				{
					printf("DCD (Data Carrier Detect)\n");
					break;
				}
				case TIOCM_RI:
				{
					printf("RI (Ring Indicator)\n");
					break;
				}
				case TIOCM_DSR:
				{
					printf("DSR (Data Set Ready)\n");
					break;
				}
				default:
				{
					printf("???\n");
					break;
				}
			}
		}
	}
# endif
}

static void
com_write(uchar *buf, int size)
{
	int r, i = 0;

	while (size)
	{
		r = write(serial_fd, buf, size);
		if (r < 0)
		{
			printf("FATAL: %s(): %s (%d)\n", __FUNCTION__, strerror(errno), errno);
			sig(0);
		}
		i += r;
		size -= r;
	}
}

static void
sio_ack(ushort devno, ushort d, uchar what)
{
	usleep((BASIC_DELAY*1000)/((useconds_t)(POKEY_AVG_HZ/1000)));

	com_write(&what, sizeof(what));

	device[devno][d].status.stat &= ~(0x01|0x04);

	switch (what)
	{
		case 'E':
		{
			device[devno][d].status.stat |= 0x04;	/* last operation failed */
			break;
		}
		case 'N':
		{
			device[devno][d].status.stat |= 0x01;
			break;
		}
	}
	
	usleep((BASIC_DELAY*bt_delay*1000)/((useconds_t)(POKEY_AVG_HZ/1000)));

# ifdef SIOTRACE
	if (log_flag)
		printf("<- ACK '%c'\n", what);
# endif
}

# ifdef ULTRA
static speed_t
make_baudrate(ushort hs_index)
{
	return (speed_t)lround( pokey_hz / (double)(2 * (hs_index + pokey_const)) );
}
# endif

static void
sio_setspeed(struct termios *com, ushort ix)
{
# ifdef TIOCGSERIAL
	struct serial_struct ss;

	if (ix)
	{
		ioctl(serial_fd, TIOCGSERIAL, &ss);
		ss.flags &= ~ASYNC_SPD_MASK;
		ioctl(serial_fd, TIOCSSERIAL, &ss);

		cfsetispeed(com, siospeed[ix].speed);
		cfsetospeed(com, siospeed[ix].speed);
		if (log_flag)
			printf("Really set %d bits/sec.\n", siospeed[ix].baud);
	}
	else
	{
		if (ioctl(serial_fd, TIOCGSERIAL, &ss) == -1)
		{
			ss.flags &= ~ASYNC_SPD_MASK;
			ioctl(serial_fd, TIOCSSERIAL, &ss);
			
			cfsetispeed(com, siospeed[3].speed);
			cfsetospeed(com, siospeed[3].speed);
			if (log_flag)
				printf("Can\'t set %d bits/sec - fallback to default %d bits/sec.\n", siospeed[ix].baud, siospeed[3].baud);
		}
		else
		{
			ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
			ss.custom_divisor = lround( (double)ss.baud_base / (double)siospeed[ix].baud );
			ioctl(serial_fd, TIOCSSERIAL, &ss);
			
			cfsetispeed(com, siospeed[2].speed);
			cfsetospeed(com, siospeed[2].speed);
			if (log_flag)
				printf("Really set %d bits/sec (base=%d, divisor=%d).\n", ss.custom_divisor? ss.baud_base / ss.custom_divisor: -1, ss.baud_base, ss.custom_divisor);
		}
	}
# else
	cfsetispeed(com, siospeed[ix].speed);
	cfsetospeed(com, siospeed[ix].speed);
# endif
	(void)tcsetattr(serial_fd, TCSANOW, com);
}

# ifdef ULTRA
static void
turbo(struct termios *com, const ushort enable)
{
	turbo_on = enable;
	sio_setspeed(com, enable ? turbo_ix : 1);
# ifdef SIOTRACE
	if (log_flag)
		printf("SIO notice: turbo %s\n", enable ? "enabled" : "disabled");
# endif
}

static void
sio_send_data_byte(ushort devno, ushort d, uchar what)
{
	sio_ack(devno, d, 'A');
	sio_ack(devno, d, 'C');
	
	outbuf[0] = outbuf[1] = what;	/* actual data and checksum */

	com_write(outbuf, 2);
}
# endif /* ULTRA */

/* ================ ATR file ================= */
static void
atr_close(ushort d)
{
	if (device[3][d].fd > -1)
		close(device[3][d].fd);

	device_reset(3, d);
}

/* Open the silly ATR file */
static int
atr_create(ushort d, const char *fname)
{
	int fd;

	fd = creat(fname, S_IRUSR|S_IWUSR);

	if (fd < 0)
		return fd;

	close(fd);

	fd = open(fname, O_RDWR|O_EXCL);

	if (fd < 0)
		return fd;

	device[3][d].fd = fd;
	device[3][d].atr.sig = SSWAP(0x0296);

	return fd;
}

static int
atr_open(char *fname, int full13force)
{
	int fd, r;
	ushort d = drvcnt;
	ulong size;
	long sl;
	ATR atr;
	struct stat sb;

	bzero(&atr, sizeof(ATR));

	atr_close(d);

	sl = strlen(fname);

	if (fname[sl - 1] == '/')
		fname[sl - 1] = 0;

	if ((r = stat(fname, &sb)) < 0)
	{
		printf("Error: %s() cannot stat() '%s', %s (%d)\n", __FUNCTION__, fname, strerror(errno), errno);
		return -1;
	}

	if ((sb.st_mode & S_IFMT) == S_IFREG)
	{
		if (drvcnt > 15)
			return -1;

		if ((fd = open(fname, O_RDWR)) < 0 &&
			( (errno != EACCES && errno != EROFS) ||
			  (fd = open(fname, O_RDONLY)) < 0 ) )
			return -1;

# if 0
		if (flock(fd, LOCK_EX|LOCK_NB) < 0)
			goto error;
# endif

		device[3][d].fd = fd;
		device[3][d].full13force = full13force;
		if (log_flag)
			printf("Disk %ld will be forced to %s after format\n", (long)d, full13force? "FULL13": "NORMAL");

		if ((read(fd, &atr, sizeof(ATR)) < (int)sizeof(ATR)) || (atr.sig != SSWAP(0x0296)))
			goto error;

		if ((atr.bps != SSWAP(0x0080)) && \
			(atr.bps != SSWAP(0x0100)) && \
				(atr.bps != SSWAP(0x0200)) && \
					(atr.bps != SSWAP(0x0400)))
		{
			goto error;
		}

		device[3][d].atr.sig = SSWAP(atr.sig);
		device[3][d].atr.wpars = SSWAP(atr.wpars);
		device[3][d].atr.bps = SSWAP(atr.bps);
		device[3][d].atr.hipars = atr.hipars;
		device[3][d].atr.crc = LSWAP(atr.crc);
		device[3][d].atr.costam = LSWAP(atr.costam);
		device[3][d].atr.prot = atr.prot;

		size = device[3][d].atr.wpars + (device[3][d].atr.hipars * 65536);
		size *= 16;

		if (drive_setup(d, size, device[3][d].atr.bps) < 0)
			goto error;

		printf("D%d: %ld sectors, %ld bytes total, mounted on %s\n", d, device[3][d].maxsec, size, fname);

		report_percom(d);

		setup_status(d);

		drvcnt++;
	}
	else
	{
		char newpath[1024];

		if ((sb.st_mode & S_IFMT) == S_IFDIR)
		{
			char oldpath[1024];

			if (pclcnt > 15)
				return -1;

			(void)getcwd(oldpath, sizeof(oldpath));
			if (chdir(fname) < 0)
				return -1;
			(void)getcwd(newpath, sizeof(newpath));
			(void)chdir(oldpath);

# if 0
			sl = strlen(newpath);

			if (sl && (newpath[sl-1] != '/'))
				strcat(newpath, "/");
# endif

			strcpy(device[6][pclcnt].dirname, newpath);
			device[6][pclcnt].on = 1;
		}
		else
			return -1;

		printf("PCL%d: mounted on %s\n", pclcnt, newpath);
		pclcnt++;
	}

	return 0;

error:	atr_close(d);

	printf("Error: %s is not a valid ATR file\n", fname);

	return -1;
}

static int
atr_seek(ushort i, long sector)
{
	long r, off;
	ushort bps = device[3][i].bps;

	/* See the info about boot sectors in DD above.
	 */
	off = (sector - 1) * bps;

	if (bps == 256)
	{
		if (device[3][i].full13)
		{
			if (sector < 4)
				off = (sector - 1) * 128;
		}
		else
		{
			if (sector < 4)
				off = (sector - 1) * 128;
			else
				off = ((sector - 4) * bps) + 384;
		}
	}

	off += 16;

	r = lseek(device[3][i].fd, off, SEEK_SET);

	if (r != off)
	{
		printf("Error: lseek() failed newpos = %ld, r = %ld\n", off, r);

		return -1;
	}

	return 0;
}

/* Format */
static void
format_atr(ushort d, int no_delay)
{
	int r, pars, cs, s, i;
	ushort spt = device[3][d].percom.spt_hi * 256 + device[3][d].percom.spt_lo;
	ushort trk = device[3][d].percom.trk, bps = device[3][d].bps;
	uchar ck;
	ATR atr;

	/* these should be suppressed when calling from make_atr() */
	if (d)
	{
		if (device[3][d].percom.trk == 1)
		{
			sio_ack(3, d, 'N');
			return;
		}
		sio_ack(3, d, 'A');
	}
	
	bzero(outbuf, sizeof(outbuf));

	if (device[3][d].percom.flags & 0x08)
		spt += (device[3][d].percom.heads * 65536);
	else
	{
		if (trk == 40 || trk == 80 || trk == 77)
			trk *= (device[3][d].percom.heads + 1);
	}

	device[3][d].full13 = device[3][d].full13force;

	pars = !device[3][d].full13force && device[3][d].bps == 256?
		device[3][d].maxsec * device[3][d].bps - 3 * 128:
		device[3][d].maxsec * device[3][d].bps;

	pars /= 16;

	device[3][d].atr.wpars = pars % 65536;
	device[3][d].atr.bps = device[3][d].bps;
	device[3][d].atr.hipars = pars / 65536;
	device[3][d].atr.crc = 0;			/* XXX */

	r = lseek(device[3][d].fd, 0, SEEK_SET);

	if (r < 0)
		goto error;

	r = ftruncate(device[3][d].fd, 0);
	
	if (r < 0)
		goto error;

	bzero(&atr, sizeof(ATR));

	atr.sig = SSWAP(device[3][d].atr.sig);
	atr.wpars = SSWAP(device[3][d].atr.wpars);
	atr.bps = SSWAP(device[3][d].atr.bps);
	atr.hipars = device[3][d].atr.hipars;
	atr.crc = LSWAP(device[3][d].atr.crc);
	atr.costam = LSWAP(device[3][d].atr.costam);
	atr.prot = device[3][d].atr.prot;

	r = write(device[3][d].fd, &atr, sizeof(ATR));

	if (r < 0)
		goto error;

	atr_seek(d, 1);

	for (i = 0; i < trk; i++)
	{
		for (s = 1; s <= spt; s++)
		{
			bps = device[3][d].bps;

			if ((i == 0) && (s < 4) && (bps == 256) && !device[3][d].full13force)
				bps = 128;

			/* seek taking into account the physical size of the bootsectors */ 
			cs = (i * spt) + s;
			r = atr_seek(d, cs);
			if (r < 0)
				goto fterror;

			r = write(device[3][d].fd, outbuf, bps);

			if (r < bps)
				goto fterror;
		}
		if (no_delay == 0)
		{
			usleep(12500);			/* ;-) */
			printf("\x7"); fflush(stdout);
		}
	}

	setup_status(d);

	outbuf[0] = 0xff;
	outbuf[1] = 0xff;

	ck = calc_checksum(outbuf, bps);

	if (d)
	{
		sio_ack(3, d, 'C');
		com_write(outbuf, bps);
		com_write(&ck, 1);
	}

	return;

fterror:
	
	if (d)
		sio_ack(3, d, 'E');

	printf("SIO write error: format failed, track %d, sector %d\n", trk, s + 1);

	return;

error:
	
	if (d)
		sio_ack(3, d, 'E');

	printf("Error: %s() failed\n", __FUNCTION__);
}

static int
make_atr(char *newname, int ch, int trk, int spt, int hds, int bps, int full13force)
{
	int r;
	uchar lpc[8];

	bzero(lpc, sizeof(lpc));

	printf("\nCreating an ATR image `%s'\n\n", newname);

	device_reset(3, 0);
	device[3][0].full13force = full13force;

	r = atr_create(0, newname);
	if (r < 0)
		return r;

	memcpy(lpc, percom_ed, 8UL);

	if (ch == 1)
	{
		lpc[3] = 0x12;
		lpc[5] = 0x00;
	}
	//else if (ch == 2)
	//	;
	else if (ch == 3)
	{
		memcpy(lpc, percom_qd, 8);
		lpc[4] = 0x00;
	}
	else if (ch == 4)
		memcpy(lpc, percom_qd, 8);
	else if (ch == 5)
	{
		memcpy(lpc, percom_qd, 8);
		lpc[0] = 80;
	}
	else if (ch == 6)
	{
		memcpy(lpc, percom_qd, 8);
		lpc[0] = 80;
		lpc[3] = 36;
	}
	else if (ch == 7)
		memcpy(lpc, percom_hd, 8);
	else if (ch == 8)
		memcpy(lpc, percom_hd32, 8);
	else if (ch == 9)
	{
		long flg = 0;

		lpc[1] = 3;

		if (spt < 65536)
		{
			if (hds > 0)
				hds--;
		}
		else
		{
			hds = (spt * trk) / 65536;
			spt -= (hds * 65536);
			flg = 0x0c;
		}

		if (bps > 128 || spt > 18)
			flg |= 0x04;			

		if (bps != 128)
		{
			if (bps & 0x00ff || bps > 0x8000)
			{
				printf("Invalid BPS value %ld\n", (long)bps);
				return -1;
			}
		}

		lpc[0] = trk;
		lpc[2] = spt / 256;
		lpc[3] = spt % 256;
		lpc[4] = hds;
		lpc[5] = flg;
		lpc[6] = bps / 256;
		lpc[7] = bps % 256;
	}

	setup_percom(0, lpc);

	format_atr(0, 1);

	atr_close(0);

	return 0;
}


/* ============= SIO COMMANDS =============== */

/* Status */
static void
sio_send_status(ushort devno, ushort d)
{
	usleep((BASIC_DELAY*1000)/((useconds_t)(POKEY_AVG_HZ/1000)));

	sio_ack(devno, d, 'A');

	setup_status(d);

	outbuf[0] = device[devno][d].status.stat;
	outbuf[1] = device[devno][d].status.err;
	outbuf[2] = device[devno][d].status.tmot;
	outbuf[3] = device[devno][d].status.none;
	outbuf[4] = calc_checksum(outbuf, 4);

	usleep((BASIC_DELAY*1000)/((useconds_t)(POKEY_AVG_HZ/1000)));

	sio_ack(devno, d, 'C');

	usleep((BASIC_DELAY*1000)/((useconds_t)(POKEY_AVG_HZ/1000)));

	com_write(outbuf, 5);
# ifdef SIOTRACE
	if (log_flag)
		printf("<- STATUS $%02x $%02x $%02x $%02x\n", outbuf[0], outbuf[1], outbuf[2], outbuf[3]);
# endif
}

/* PERCOM block */
static void
send_percom(ushort d)
{
	sio_ack(3, d, 'A');

	outbuf[0] = device[3][d].percom.trk;
	outbuf[1] = device[3][d].percom.step;
	outbuf[2] = device[3][d].percom.spt_hi;
	outbuf[3] = device[3][d].percom.spt_lo;
	outbuf[4] = device[3][d].percom.heads;
	outbuf[5] = device[3][d].percom.flags;
	outbuf[6] = device[3][d].percom.bps_hi;
	outbuf[7] = device[3][d].percom.bps_lo;
	outbuf[8] = 0xff;
	outbuf[9] = 0x00;
	outbuf[10] = 0x00;
	outbuf[11] = 0x00;

	outbuf[12] = calc_checksum(outbuf, 12);

	sio_ack(3, d, 'C');

	com_write(outbuf, 13);
# ifdef SIOTRACE
	if (log_flag)
		printf("<- PERCOM\n");
# endif
}

static void
receive_percom(ushort d)
{
	uchar ck;
	int r;

# if 0
	if (device[3][d].percom.trk == 1)
	{
		sio_ack(3, d, 'N');
		return;
	}
# endif
	
	sio_ack(3, d, 'A');

	device[3][d].status.stat &= ~0x02;

	com_read(inpbuf, 13, COM_DATA);

	ck = calc_checksum(inpbuf, 12);

	if (ck != inpbuf[12])
	{
		device[3][d].status.stat |= 0x02;
		return;
	}

	if (device[3][d].percom.trk == 1)
		r = 0;
	else
		r = setup_percom(d, inpbuf);

	if (r == 0)
	{
		sio_ack(3, d, 'A');

		setup_status(d);

		sio_ack(3, d, 'C');

# ifdef SIOTRACE
		if (log_flag)
		{
			printf("-> PERCOM: %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x\n", \
				device[3][d].percom.trk, device[3][d].percom.step,
				device[3][d].percom.spt_hi, device[3][d].percom.spt_lo,
				device[3][d].percom.heads, device[3][d].percom.flags,
				device[3][d].percom.bps_hi, device[3][d].percom.bps_lo);
		}
# endif
	}
	else
		sio_ack(3, d, 'E');
}

/* Sectors */
static void
send_sector(uchar devno, int i, uchar ccom, long sector)
{
	uchar ck = 0;
	ushort bps = device[devno][i].bps;

	if ((devno == 3) && ((sector == 0) || (sector > (long)device[3][i].maxsec)))
	{
		sio_ack(devno, i, 'N');
		return;
	}

	sio_ack(devno, i, 'A');

	if ((devno == 3) && (bps == 256) && (sector < 4))
		bps = 128;

	if (devno == 3)
		if (atr_seek(i, sector) < 0)
			goto error;
	if (read(device[devno][i].fd, outbuf, bps) < bps)
		goto error;

	sio_ack(devno, i, 'C');

	if (ccom != 'V')
	{
		ck = outbuf[bps] = calc_checksum(outbuf, bps);
		com_write(outbuf, (bps + 1));
	}

# ifdef SIOTRACE
	if (log_flag)
		printf("<- SECTOR $%04lx (%5ld), bps: %d, CRC: $%02x\n", sector, sector, bps, ck);
# endif

	return;

error:
	sio_ack(devno, i, 'E');

	/* SIO expects the transfer even after Error was signalized */

	if (ccom != 'V')
	{
		outbuf[bps] = calc_checksum(outbuf, bps);
		com_write(outbuf, (bps + 1));
	}

	printf("SIO read error: D%d:, sector $%04lx (%5ld), bps: %d\n", i, sector, sector, bps);
}

static void
receive_sector(ushort devno, ushort i, long sector)
{
	ushort bps = device[devno][i].bps;
	uchar ck, sck;

	if ((devno == 3) && i && ((sector == 0) || (sector > (long)device[3][i].maxsec)))
	{
		sio_ack(devno, i, 'N');
		return;
	}

	sio_ack(devno, i, 'A');

	if ((devno == 3) && (bps == 256) && (sector < 4))
		bps = 128;

	com_read(inpbuf, bps, COM_DATA);

	ck = calc_checksum(inpbuf, bps);

	device[devno][i].status.stat &= ~0x02;

	com_read(&sck, 1, COM_DATA);

	if (ck != sck)
	{
		device[devno][i].status.stat |= 0x02;
		printf("SIO write: CRC fail, Atari: $%02x, PC: $%02x\n", sck, ck);
		goto error;
	}

# ifdef SIOTRACE
	if (log_flag)
		printf("-> SECTOR $%04lx (%5ld), bps: %d, CRC: $%02x\n", sector, sector, bps, ck);
# endif

	sio_ack(devno, i, 'A');

	if (devno == 3)
		if (atr_seek(i, sector) < 0)
			goto error;
	if (device[devno][i].fd > -1)
		if (write(device[devno][i].fd, inpbuf, bps) != bps)
			goto error;

	sio_ack(devno, i, 'C');

	return;

error:	sio_ack(devno, i, 'E');

	printf("SIO write error: D%d:, sector $%04lx (%5ld), bps: %d\n", i, sector, sector, bps);
}

static void
sig(int s)
{
	int i;

	if (s)
	{
# ifdef __CYGWIN__
		printf("Caught signal %d\n", s);
# else
		psignal(s, NULL);
# endif
	}

	if (printer_fd > -1)
		close(printer_fd);
	
	if (serial_fd > -1)
	{
		(void)tcsetattr(serial_fd, TCSANOW, &dflt);
		close(serial_fd);
	}
	
	for (i = 0; i < 15; i++)
		atr_close(i);

	(void)unlink(dpath);
	
	exit(s);
}

/* PCLink part */

# define SDX_MAXLEN 16777215L

/* SDX required attribute mask */
# define RA_PROTECT	0x01
# define RA_HIDDEN	0x02
# define RA_ARCHIVED	0x04
# define RA_SUBDIR	0x08
# define RA_NO_PROTECT	0x10
# define RA_NO_HIDDEN	0x20
# define RA_NO_ARCHIVED	0x40
# define RA_NO_SUBDIR	0x80

/* SDX set attribute mask */
# define SA_PROTECT	0x01
# define SA_UNPROTECT	0x10
# define SA_HIDE	0x02
# define SA_UNHIDE	0x20
# define SA_ARCHIVE	0x04
# define SA_UNARCHIVE	0x40
# define SA_SUBDIR	0x08	/* illegal mode */
# define SA_UNSUBDIR	0x80	/* illegal mode */

# define DEVICE_LABEL	".PCLINK.VOLUME.LABEL"

# define PCL_MAX_FNO	0x14

static const char *fun[] =
{
	"FREAD", "FWRITE", "FSEEK", "FTELL", "FLEN", "(none)", "FNEXT", "FCLOSE",
	"INIT", "FOPEN", "FFIRST", "RENAME", "REMOVE", "CHMOD", "MKDIR", "RMDIR",
	"CHDIR", "GETCWD", "SETBOOT", "DFREE", "CHVOL"
};

typedef struct
{
	uchar status;
	uchar map_l, map_h;
	uchar len_l, len_m, len_h;
	char fname[11];
	uchar stamp[6];
} DIRENTRY;

static struct
{
	union
	{
		FILE *file;
		DIR *dir;
	} fps;

	DIRENTRY *dir_cache;	/* used only for directories */

	uchar devno;
	uchar cunit;
	uchar fpmode;
	uchar fatr1;
	uchar fatr2;
	uchar t1,t2,t3;
	uchar d1,d2,d3;
	struct stat fpstat;
	char fpname[12];
	long fppos;
	long fpread;
	int eof;
	char pathname[1024];
} iodesc[16];

static struct
{
	uchar handle;
	uchar dirbuf[23];
} pcl_dbf;

static ulong upper_dir = UPPER_DIR;

static long
dos_2_allowed(uchar c)
{
# ifndef __CYGWIN__
	if (upper_dir)
		return (isupper(c) || isdigit(c) || (c == '_') || (c == '@'));

	return (islower(c) || isdigit(c) || (c == '_') || (c == '@'));
# else
	return (isalpha(c) || isdigit(c) || (c == '_') || (c == '@'));
# endif
}

static long
dos_2_term(uchar c)
{
	return ((c == 0) || (c == 0x20));
}

static long
validate_fn(uchar *name, int len)
{
	int x;

	for (x = 0; x < len; x++)
	{
		if (dos_2_term(name[x]))
			return (x != 0);
		if (name[x] == '.')
			return 1;
		if (!dos_2_allowed(name[x]))
			return 0;
	}

	return 1;
}

static void
ugefina(char *src, char *out)
{
	char *dot;
	ushort i;

	memset(out, 0x20, 8+3);

	dot = strchr(src, '.');

	if (dot)
	{
		i = 1;
		while (dot[i] && (i < 4))
		{
			out[i+7] = toupper((uchar)dot[i]);
			i++;
		}
	}

	i = 0;
	while ((src[i] != '.') && !dos_2_term(src[i]) && (i < 8))
	{
		out[i] = toupper((uchar)src[i]);
		i++;
	}
}

static void
uexpand(uchar *rawname, char *name83)
{
	ushort x, y;
	uchar t;

	name83[0] = 0;

	for (x = 0; x < 8; x++)
	{
		t = rawname[x];
		if (t && (t != 0x20))
			name83[x] = upper_dir ? toupper(t) : tolower(t);
		else
			break;
	}

	y = 8;

	if (rawname[y] && (rawname[y] != 0x20))
	{
		name83[x] = '.';
		x++;

		while ((y < 11) && rawname[y] && (rawname[y] != 0x20))
		{
			name83[x] = upper_dir ? toupper(rawname[y]) : tolower(rawname[y]);
			x++;
			y++;
		}
	}

	name83[x] = 0;
}

static int
match_dos_names(char *name, char *mask, uchar fatr1, struct stat *sb)
{
	ushort i;

	if (log_flag)
	{
		printf("match: %c%c%c%c%c%c%c%c%c%c%c with %c%c%c%c%c%c%c%c%c%c%c: ",
		name[0], name[1], name[2], name[3], name[4], name[5], name[6], name[7], \
		name[8], name[9], name[10], \
		mask[0], mask[1], mask[2], mask[3], mask[4], mask[5], mask[6], mask[7], \
		mask[8], mask[9], mask[10]);
	}

	for (i = 0; i < 11; i++)
	{
		if (mask[i] != '?')
			if (toupper((uchar)name[i]) != toupper((uchar)mask[i]))
			{
				if (log_flag)
					printf("no match\n");
				return 1;
			}
	}

	/* There are no such attributes in Unix */
	fatr1 &= ~(RA_NO_HIDDEN|RA_NO_ARCHIVED);

	/* Now check the attributes */
	if (fatr1 & (RA_HIDDEN | RA_ARCHIVED))
	{
		if (log_flag)
			printf("atr mismatch: not HIDDEN or ARCHIVED\n");
		return 1;
	}

	if (fatr1 & RA_PROTECT)
	{
		if (sb->st_mode & S_IWUSR)
		{
			if (log_flag)
				printf("atr mismatch: not PROTECTED\n");

			return 1;
		}
	}

	if (fatr1 & RA_NO_PROTECT)
	{
		if ((sb->st_mode & S_IWUSR) == 0)
		{
			if (log_flag)
				printf("atr mismatch: not UNPROTECTED\n");

			return 1;
		}
	}

	if (fatr1 & RA_SUBDIR)
	{
		if (!S_ISDIR(sb->st_mode))
		{
			if (log_flag)
				printf("atr mismatch: not SUBDIR\n");

			return 1;
		}
	}

	if (fatr1 & RA_NO_SUBDIR)
	{
		if (S_ISDIR(sb->st_mode))
		{
			if (log_flag)
				printf("atr mismatch: not FILE\n");

			return 1;
		}
	}

	if (log_flag)
		printf("match\n");

	return 0;
}

static int
validate_dos_name(char *fname)
{
	char *dot = strchr(fname, '.');
	long valid_fn, valid_xx;
			
	if ((dot == NULL) && (strlen(fname) > 8))
		return 1;
	if (dot)
	{
		long dd = strlen(dot);

		if (dd > 4)
			return 1;
		if ((dot - fname) > 8)
			return 1;
		if ((dot == fname) && (dd == 1))
			return 1;
		if ((dd == 2) && (dot[1] == '.'))
			return 1;
		if ((dd == 3) && ((dot[1] == '.') || (dot[2] == '.')))
			return 1;
		if ((dd == 4) && ((dot[1] == '.') || (dot[2] == '.') || (dot[3] == '.')))
			return 1;
	}

	valid_fn = validate_fn((uchar *)fname, 8);
	if (dot != NULL)
		valid_xx = validate_fn((uchar *)(dot + 1), 3);
	else
		valid_xx = 1;

	if (!valid_fn || !valid_xx)
		return 1;

	return 0;
}

static int
check_dos_name(char *newpath, struct dirent *dp, struct stat *sb)
{
	char temp_fspec[1024], fname[256];

	strcpy(fname, dp->d_name);

	if (log_flag)
		printf("%s: got fname '%s'\n", __FUNCTION__, fname); 

	if (validate_dos_name(fname))
		return 1;

	/* stat() the file (fetches the length) */
	sprintf(temp_fspec, "%s/%s", newpath, fname);

	if (log_flag)
		printf("%s: stat '%s'\n", __FUNCTION__, temp_fspec);

	if (stat(temp_fspec, sb))
		return 1;

	if (!S_ISREG(sb->st_mode) && !S_ISDIR(sb->st_mode))
		return 1;

	if (sb->st_uid != our_uid)		/* belongs to us? */
		return 1;

	if ((sb->st_mode & S_IRUSR) == 0)	/* unreadable? */
		return 1;

	if (S_ISDIR(sb->st_mode) && ((sb->st_mode & S_IXUSR) == 0))
		return 1;

	return 0;
}

static void
fps_close(int i)
{
	if (iodesc[i].fps.file != NULL)
	{
		if (iodesc[i].fpmode & 0x10)
			closedir(iodesc[i].fps.dir);
		else
			fclose(iodesc[i].fps.file);
	}

	if (iodesc[i].dir_cache != NULL)
	{
		free(iodesc[i].dir_cache);
		iodesc[i].dir_cache = NULL;
	}

	iodesc[i].fps.file = NULL;

	iodesc[i].devno = 0;
	iodesc[i].cunit = 0;
	iodesc[i].fpmode = 0;
	iodesc[i].fatr1 = 0;
	iodesc[i].fatr2 = 0;
	iodesc[i].t1 = 0;
	iodesc[i].t2 = 0;
	iodesc[i].t3 = 0;
	iodesc[i].d1 = 0;
	iodesc[i].d2 = 0;
	iodesc[i].d3 = 0;
	iodesc[i].fpname[0] = 0;
	iodesc[i].fppos = 0;
	iodesc[i].fpread = 0;
	iodesc[i].eof = 0;
	iodesc[i].pathname[0] = 0;
	bzero(&iodesc[i].fpstat, sizeof(struct stat));
}

static ulong
get_file_len(uchar handle)
{
	ulong filelen;
	struct dirent *dp;
	struct stat sb;

	if (iodesc[handle].fpmode & 0x10)	/* directory */
	{
		rewinddir(iodesc[handle].fps.dir);
		filelen = sizeof(DIRENTRY);

		while ((dp = readdir(iodesc[handle].fps.dir)) != NULL)
		{
			if (check_dos_name(iodesc[handle].pathname, dp, &sb))
				continue;
			filelen += sizeof(DIRENTRY);
		}
		rewinddir(iodesc[handle].fps.dir);
	}
	else
		filelen = iodesc[handle].fpstat.st_size;

	if (filelen > SDX_MAXLEN)
		filelen = SDX_MAXLEN;

	return filelen;
}

static DIRENTRY *
cache_dir(uchar handle)
{
	char *bs, *cwd;
	uchar dirnode = 0x00;
	ushort node;
	ulong dlen, flen, sl, dirlen = iodesc[handle].fpstat.st_size;
	DIRENTRY *dbuf, *dir;
	struct dirent *dp;
	struct stat sb;

	if (iodesc[handle].dir_cache != NULL)
	{
		printf("Internal error: dir_cache should be NULL!\n");
		sig(0);
	}

	dir = dbuf = malloc(dirlen + sizeof(DIRENTRY));
	bzero(dbuf, dirlen + sizeof(DIRENTRY));

	dir->status = 0x28;
	dir->map_l = 0x00;			/* low 11 bits: file number, high 5 bits: dir number */
	dir->map_h = dirnode;
	dir->len_l = dirlen & 0x000000ffL;
	dir->len_m = (dirlen & 0x0000ff00L) >> 8;
	dir->len_h = (dirlen & 0x00ff0000L) >> 16;

	memset(dir->fname, 0x20, 11);

	sl = strlen(device[iodesc[handle].devno][iodesc[handle].cunit].dirname);

	cwd = iodesc[handle].pathname + sl;

	bs = strrchr(cwd, '/');

	if (bs == NULL)
		memcpy(dir->fname, "MAIN", 4);
	else
	{
		char *cp = cwd;

		ugefina(bs+1, (char *)dir->fname);

		node = 0;

		while (cp <= bs)
		{
			if (*cp == '/')
				dirnode++;
			cp++;
		}

		dir->map_h = (dirnode & 0x1f) << 3;
	}

	unix_time_2_sdx(&iodesc[handle].fpstat.st_mtime, dir->stamp);

	dir++;
	flen = sizeof(DIRENTRY);

	node = 1;

	while ((dp = readdir(iodesc[handle].fps.dir)) != NULL)
	{
		ushort map;

		if (check_dos_name(iodesc[handle].pathname, dp, &sb))
			continue;

		dlen = sb.st_size;
		if (dlen > SDX_MAXLEN)
			dlen = SDX_MAXLEN;

		dir->status = (sb.st_mode & S_IWUSR) ? 0x08 : 0x09;

		if (S_ISDIR(sb.st_mode))
		{
			dir->status |= 0x20;		/* directory */
			dlen = sizeof(DIRENTRY);
		}

		map = dirnode << 11;
		map |= (node & 0x07ff);

		dir->map_l = map & 0x00ff;
		dir->map_h = ((map & 0xff00) >> 8);
		dir->len_l = dlen & 0x000000ffL;
		dir->len_m = (dlen & 0x0000ff00L) >> 8;
		dir->len_h = (dlen & 0x00ff0000L) >> 16;

		ugefina(dp->d_name, (char *)dir->fname);

		unix_time_2_sdx(&sb.st_mtime, dir->stamp);

		node++;
		dir++;
		flen += sizeof(DIRENTRY);

		if (flen >= dirlen)
			break;
	}

	return dbuf;
}

static ulong
dir_read(uchar *mem, ulong blk_size, uchar handle, int *eof_sig)
{
	uchar *db = (uchar *)iodesc[handle].dir_cache;
	ulong dirlen = iodesc[handle].fpstat.st_size, newblk;

	eof_sig[0] = 0;

	newblk = dirlen - iodesc[handle].fppos;

	if (newblk < blk_size)
	{
		blk_size = newblk; 
		eof_sig[0] = 1;
	}

	if (blk_size)
		memcpy(mem, db+iodesc[handle].fppos, blk_size);

	return blk_size;
}

static void
do_pclink_init(int force)
{
	uchar handle;

	if (force == 0)
		printf("closing all files\n");

	for (handle = 0; handle < 16; handle++)
	{
		if (force)
			iodesc[handle].fps.file = NULL;
		fps_close(handle);
		bzero(&device[6][handle].parbuf, sizeof(PARBUF));
	}
}

static void
set_status_size(uchar devno, uchar cunit, ushort size)
{
	device[devno][cunit].status.tmot = (size & 0x00ff);
	device[devno][cunit].status.none = (size & 0xff00) >> 8;
}

static int
validate_user_path(char *defwd, char *newpath)
{
	char *d, oldwd[1024], newwd[1024];

	(void)getcwd(oldwd, sizeof(oldwd));
	if (chdir(newpath) < 0)
		return 0;
	(void)getcwd(newwd, sizeof(newwd));
	(void)chdir(oldwd);

	d = strstr(newwd, defwd);

	if (d == NULL)
		return 0;
	if (d != newwd)
		return 0;

	return 1;
}

static int
ispathsep(uchar c)
{
	return ((c == '>') || (c == '\\'));
}

static void
path_copy(uchar *dst, uchar *src)
{
	uchar a;

	while (*src)
	{
		a = *src;
		if (ispathsep(a))
		{
			while (ispathsep(*src))
				src++;
			src--;
		}
		*dst = a;
		src++;
		dst++;
	}

	*dst = 0;
}

static void
path2unix(uchar *out, uchar *path)
{
	int i, y = 0;

	for (i = 0; path[i] && (i < 64); i++)
	{
		char a;

		a = upper_dir ? toupper(path[i]) : tolower(path[i]);

		if (ispathsep(a))
			a = '/';
		else if (a == '<')
		{
			a = '.';
			out[y++] = '.';
		}
		out[y++] = a;
	}

	if (y && (out[y-1] != '/'))
		out[y++] = '/';

	out[y] = 0;
}

static void
create_user_path(uchar devno, uchar cunit, char *newpath)
{
	long sl, cwdo = 0;
	uchar lpath[128], upath[128];

	strcpy(newpath, device[devno][cunit].dirname);

	/* this is user-requested new path */
	path_copy(lpath, device[devno][cunit].parbuf.path);
	path2unix(upath, lpath);

	if (upath[0] != '/')
	{
		sl = strlen(newpath);
		if (sl && (newpath[sl-1] != '/'))
			strcat(newpath, "/");
		if (device[devno][cunit].cwd[0] == '/')
			cwdo++;
		strcat(newpath, (char *)device[devno][cunit].cwd + cwdo);
		sl = strlen(newpath);
		if (sl && (newpath[sl-1] != '/'))
			strcat(newpath, "/");
	}
	strcat(newpath, (char *)upath);
	sl = strlen(newpath);
	if (sl && (newpath[sl-1] == '/'))
		newpath[sl-1] = 0;
}

static time_t
timestamp2mtime(uchar *stamp)
{
	struct tm sdx_tm;

	bzero(&sdx_tm, sizeof(struct tm));

	sdx_tm.tm_sec = stamp[5];
	sdx_tm.tm_min = stamp[4];
	sdx_tm.tm_hour = stamp[3];
	sdx_tm.tm_mday = stamp[0];
	sdx_tm.tm_mon = stamp[1];
	sdx_tm.tm_year = stamp[2];

	if ((sdx_tm.tm_mday == 0) || (sdx_tm.tm_mon == 0))
		return 0;

	if (sdx_tm.tm_mon)
		sdx_tm.tm_mon--;

	if (sdx_tm.tm_year < 80)
		sdx_tm.tm_year += 2000;
	else
		sdx_tm.tm_year += 1900;

	sdx_tm.tm_year -= 1900;

	return mktime(&sdx_tm);
}

/* Command: DDEVIC+DUNIT-1 = $6f, DAUX1 = parbuf size, DAUX2 = %vvvvuuuu
 * where: v - protocol version number (0), u - unit number
 */

static void
do_pclink(uchar devno, uchar ccom, uchar caux1, uchar caux2)
{
	uchar ck, sck, fno, ob[7], handle;
	ushort cunit = caux2 & 0x0f, parsize;
	ulong faux;
	struct stat sb;
	struct dirent *dp;
	static uchar old_ccom = 0;

	parsize = caux1 ? caux1 : 256;

	if (caux2 & 0xf0)	/* protocol version number must be 0 */
	{
		sio_ack(devno, cunit, 'N');
		return;
	}

	if (parsize > sizeof(PARBUF))	/* and not more than fits in parbuf */
	{
		sio_ack(devno, cunit, 'N');
		return;
	}

	if (ccom == 'P')
	{
		PARBUF pbuf;

		sio_ack(devno, cunit, 'A');	/* ack the command */

		bzero(&pbuf, sizeof(PARBUF));

		com_read((uchar *)&pbuf, parsize, COM_DATA);
		com_read(&sck, 1, COM_DATA);

		ck = calc_checksum((uchar *)&pbuf, parsize);

		device[devno][cunit].status.stat &= ~0x02;

		sio_ack(devno, cunit, 'A');	/* ack the received block */

		if (ck != sck)
		{
			device[devno][cunit].status.stat |= 0x02;
			printf("PARBLK CRC error, Atari: $%02x, PC: $%02x\n", sck, ck);
			device[devno][cunit].status.err = 143;
			goto complete;
		}

		device[devno][cunit].status.stat &= ~0x04;

# if 0
		/* True if Atari didn't catch the ACK above and retried the command */
		if (pbuf.fno > PCL_MAX_FNO)
		{
			device[devno][cunit].status.stat |= 0x04;
			printf("PARBLK error, invalid fno $%02x\n", pbuf.fno);
			device[devno][cunit].status.err = 144;
			goto complete;
		}
# endif

		if (memcmp(&pbuf, &device[devno][cunit].parbuf, sizeof(PARBUF)) == 0)
		{
			/* this is a retry of P-block. Most commands don't like that */
			if ((pbuf.fno != 0x00) && (pbuf.fno != 0x01) && (pbuf.fno != 0x03) \
				&& (pbuf.fno != 0x04) && (pbuf.fno != 0x06) && \
					(pbuf.fno != 0x11) && (pbuf.fno != 0x13))
			{
				printf("PARBLK retry, ignored\n");
				goto complete;
			}
		}

		memcpy(&device[devno][cunit].parbuf, &pbuf, sizeof(PARBUF));
	}

//	device[devno][cunit].status.err = 1;
//	set_status_size(devno, cunit, 0);

	fno = device[devno][cunit].parbuf.fno;
	faux = device[devno][cunit].parbuf.f1 + device[devno][cunit].parbuf.f2 * 256 + \
		device[devno][cunit].parbuf.f3 * 65536;

	if (fno < (PCL_MAX_FNO+1))
		printf("%s (fno $%02x): ", fun[fno], fno);

	handle = device[devno][cunit].parbuf.handle;

	if (fno == 0x00)	/* FREAD */
	{
		uchar *mem;
		ulong blk_size = (faux & 0x0000FFFFL), buffer;

		if (ccom == 'P')
		{
			if ((handle > 15) || (iodesc[handle].fps.file == NULL))
			{
				printf("bad handle %d\n", handle);
				device[devno][cunit].status.err = 134;	/* bad file handle */
				goto complete;
			}

			if (blk_size == 0)
			{
				printf("bad size $0000 (0)\n");
				device[devno][cunit].status.err = 176;
				set_status_size(devno, cunit, 0);
				goto complete;
			}

			device[devno][cunit].status.err = 1;
			iodesc[handle].eof = 0;

			buffer = iodesc[handle].fpstat.st_size - iodesc[handle].fppos;

			if (buffer < blk_size)
			{
				blk_size = buffer;
				device[devno][cunit].parbuf.f1 = (buffer & 0x00ff);
				device[devno][cunit].parbuf.f2 = (buffer & 0xff00) >> 8;
				iodesc[handle].eof = 1;
				if (blk_size == 0)
					device[devno][cunit].status.err = 136;
			}

			printf("size $%04lx (%ld), buffer $%04lx (%ld)\n", blk_size, blk_size, buffer, buffer);

			set_status_size(devno, cunit, (ushort)blk_size);
			goto complete;
		}

		if ((ccom == 'R') && (old_ccom == 'R'))
		{
			sio_ack(devno, cunit, 'N');
			printf("serial communication error, abort\n");
			return;
		}

		sio_ack(devno, cunit, 'A');	/* ack the command */

		printf("handle %d\n", handle);

		mem = malloc(blk_size + 1);

		if ((device[devno][cunit].status.err == 1))
		{
			iodesc[handle].fpread = blk_size;

			if (iodesc[handle].fpmode & 0x10)
			{
				ulong rdata;
				int eof_sig;

				rdata = dir_read(mem, blk_size, handle, &eof_sig);

				if (rdata != blk_size)
				{
					printf("FREAD: cannot read %ld bytes from dir\n", blk_size);
					if (eof_sig)
					{
						iodesc[handle].fpread = rdata;
						device[devno][cunit].status.err = 136;
					}
					else
					{
						iodesc[handle].fpread = 0;
						device[devno][cunit].status.err = 255;
					}
				}
			}
			else
			{
				if (fseek(iodesc[handle].fps.file, iodesc[handle].fppos, SEEK_SET))
				{
					printf("FREAD: cannot seek to $%04lx (%ld)\n", iodesc[handle].fppos, iodesc[handle].fppos);
					device[devno][cunit].status.err = 166;
				}
				else
				{
					long fdata = fread(mem, sizeof(char), blk_size, iodesc[handle].fps.file);

					if ((ulong)fdata != blk_size)
					{
						printf("FREAD: cannot read %ld bytes from file\n", blk_size);
						if (feof(iodesc[handle].fps.file))
						{
							iodesc[handle].fpread = fdata;
							device[devno][cunit].status.err = 136;
						}
						else
						{
							iodesc[handle].fpread = 0;
							device[devno][cunit].status.err = 255;
						}
					}
				}
			}
		}

		iodesc[handle].fppos += iodesc[handle].fpread;

		if (device[devno][cunit].status.err == 1)
		{
			if (iodesc[handle].eof)
				device[devno][cunit].status.err = 136;
			else if (iodesc[handle].fppos == iodesc[handle].fpstat.st_size)
				device[devno][cunit].status.err = 3;
		}

		set_status_size(devno, cunit, iodesc[handle].fpread);

		printf("FREAD: send $%04lx (%ld), status $%02x\n", blk_size, blk_size, device[devno][cunit].status.err);

		sck = calc_checksum((void *)mem, blk_size);
		mem[blk_size] = sck;
		sio_ack(devno, cunit, 'C');
		com_write(mem, blk_size + 1);

		free(mem);

		goto exit;
	}

	if (fno == 0x01)	/* FWRITE */
	{
		uchar *mem;
		ulong blk_size = (faux & 0x0000FFFFL);

		if (ccom == 'P')
		{
			if ((handle > 15) || (iodesc[handle].fps.file == NULL))
			{
				printf("bad handle %d\n", handle);
				device[devno][cunit].status.err = 134;	/* bad file handle */
				goto complete;
			}

			if (blk_size == 0)
			{
				printf("bad size $0000 (0)\n");
				device[devno][cunit].status.err = 176;
				set_status_size(devno, cunit, 0);
				goto complete;
			}

			device[devno][cunit].status.err = 1;

			printf("size $%04lx (%ld)\n", blk_size, blk_size);
			set_status_size(devno, cunit, (ushort)blk_size);
			goto complete;
		}

		if ((ccom == 'R') && (old_ccom == 'R'))
		{
			sio_ack(devno, cunit, 'N');
			printf("serial communication error, abort\n");
			return;
		}

		sio_ack(devno, cunit, 'A');	/* ack the command */

		printf("handle %d\n", handle);

		if ((iodesc[handle].fpmode & 0x10) == 0)
		{
			if (fseek(iodesc[handle].fps.file, iodesc[handle].fppos, SEEK_SET))
			{
				printf("FWRITE: cannot seek to $%06lx (%ld)\n", iodesc[handle].fppos, iodesc[handle].fppos);
				device[devno][cunit].status.err = 166;
			}
		}

		mem = malloc(blk_size + 1);

		com_read(mem, blk_size, COM_DATA);
		com_read(&sck, sizeof(uchar), COM_DATA);

		sio_ack(devno, cunit, 'A'); 	/* ack the block of data */

		ck = calc_checksum(mem, blk_size);

		if (ck != sck)
		{
			printf("FWRITE: block CRC mismatch\n");
			device[devno][cunit].status.err = 143;
			free(mem);
			goto complete;
		}

		if (device[devno][cunit].status.err == 1)
		{
			long rdata;

			iodesc[handle].fpread = blk_size;

			if (iodesc[handle].fpmode & 0x10)
			{
				/* ignore raw dir writes */
			}
			else
			{
				rdata = fwrite(mem, sizeof(char), blk_size, iodesc[handle].fps.file);

				if ((ulong)rdata != blk_size)
				{
					printf("FWRITE: cannot write %ld bytes to file\n", blk_size);
					iodesc[handle].fpread = rdata;
					device[devno][cunit].status.err = 255;
				}
			}
		}

		iodesc[handle].fppos += iodesc[handle].fpread;

		set_status_size(devno, cunit, iodesc[handle].fpread);

		printf("FWRITE: received $%04lx (%ld), status $%02x\n", blk_size, blk_size, device[devno][cunit].status.err);

		free(mem);
		goto complete;
	}

	if (fno == 0x02)	/* FSEEK */
	{
		ulong newpos = faux;

		if ((handle > 15) || (iodesc[handle].fps.file == NULL))
		{
			printf("bad handle %d\n", handle);
			device[devno][cunit].status.err = 134;	/* bad file handle */
			goto complete;
		}

		if (ccom == 'R')
		{
			sio_ack(devno, cunit, 'A');	/* ack the command */
			printf("bad exec\n");
			device[devno][cunit].status.err = 176;
			goto complete;
		}

		device[devno][cunit].status.err = 1;

		printf("handle %d, newpos $%06lx (%ld)\n", handle, newpos, newpos);

		if (iodesc[handle].fpmode & 0x08)
			iodesc[handle].fppos = newpos;
		else
		{
			if (newpos <= iodesc[handle].fpstat.st_size)
				iodesc[handle].fppos = newpos;
			else
				device[devno][cunit].status.err = 166;
		}

		goto complete;
	}

	if ((fno == 0x03) || (fno == 0x04))	/* FTELL/FLEN */
	{
		ulong outval = 0;
		uchar out[4];

		if (ccom == 'P')
		{
			if ((handle > 15) || (iodesc[handle].fps.file == NULL))
			{
				printf("bad handle %d\n", handle);
				device[devno][cunit].status.err = 134;	/* bad file handle */
				goto complete;
			}

			device[devno][cunit].status.err = 1;

			printf("device $%02x\n", cunit);
			goto complete;
		}

		sio_ack(devno, cunit, 'A');	/* ack the command */

		if (fno == 0x03)
			outval = iodesc[handle].fppos;
		else
			outval = iodesc[handle].fpstat.st_size;

		printf("handle %d, send $%06lx (%ld)\n", handle, outval, outval);

		out[0] = (uchar)(outval & 0x000000ffL);
		out[1] = (uchar)((outval & 0x0000ff00L) >> 8);
		out[2] = (uchar)((outval & 0x00ff0000L) >> 16);

		out[3] = calc_checksum((void *)out, sizeof(out)-1);
		sio_ack(devno, cunit, 'C');
		com_write(out, sizeof(out));
		goto exit;
	}

	if (fno == 0x06)	/* FNEXT */
	{
		if (ccom == 'P')
		{
			device[devno][cunit].status.err = 1;

			printf("device $%02x\n", cunit);
			goto complete;
		}

		if ((ccom == 'R') && (old_ccom == 'R'))
		{
			sio_ack(devno, cunit, 'N');
			printf("serial communication error, abort\n");
			return;
		}

		sio_ack(devno, cunit, 'A');	/* ack the command */

		bzero(pcl_dbf.dirbuf, sizeof(pcl_dbf.dirbuf));

		if ((handle > 15) || (iodesc[handle].fps.file == NULL))
		{
			printf("bad handle %d\n", handle);
			device[devno][cunit].status.err = 134;	/* bad file handle */
		}
		else
		{
			int eof_flg, match = 0;

			printf("handle %d\n", handle);

			do
			{
				struct stat ts;

				bzero(&ts, sizeof(ts));
				bzero(pcl_dbf.dirbuf, sizeof(pcl_dbf.dirbuf));
				iodesc[handle].fppos += dir_read(pcl_dbf.dirbuf, sizeof(pcl_dbf.dirbuf), handle, &eof_flg);

				if (!eof_flg)
				{
					/* fake stat to satisfy match_dos_names() */
					if ((pcl_dbf.dirbuf[0] & 0x01) == 0)
						ts.st_mode |= S_IWUSR;
					if (pcl_dbf.dirbuf[0] & 0x20)
						ts.st_mode |= S_IFDIR;
					else
						ts.st_mode |= S_IFREG;

					match = !match_dos_names((char *)pcl_dbf.dirbuf+6, iodesc[handle].fpname, iodesc[handle].fatr1, &ts);
				}

			} while (!eof_flg && !match);

			if (eof_flg)
			{
				printf("FNEXT: EOF\n");
				device[devno][cunit].status.err = 136;
			}
			else if (iodesc[handle].fppos == iodesc[handle].fpstat.st_size)
				device[devno][cunit].status.err = 3;
		}

		/* avoid the 4th execution stage */
		pcl_dbf.handle = device[devno][cunit].status.err;

		printf("FNEXT: status %d, send $%02x $%02x%02x $%02x%02x%02x %c%c%c%c%c%c%c%c%c%c%c %02d-%02d-%02d %02d:%02d:%02d\n", \
			pcl_dbf.handle,
			pcl_dbf.dirbuf[0],
			pcl_dbf.dirbuf[2], pcl_dbf.dirbuf[1],
			pcl_dbf.dirbuf[5], pcl_dbf.dirbuf[4], pcl_dbf.dirbuf[3],
			pcl_dbf.dirbuf[6], pcl_dbf.dirbuf[7], pcl_dbf.dirbuf[8], pcl_dbf.dirbuf[9],
			pcl_dbf.dirbuf[10], pcl_dbf.dirbuf[11], pcl_dbf.dirbuf[12], pcl_dbf.dirbuf[13],
			pcl_dbf.dirbuf[14], pcl_dbf.dirbuf[15], pcl_dbf.dirbuf[16],
			pcl_dbf.dirbuf[17], pcl_dbf.dirbuf[18], pcl_dbf.dirbuf[19],
			pcl_dbf.dirbuf[20], pcl_dbf.dirbuf[21], pcl_dbf.dirbuf[22]);
		
		sck = calc_checksum((void *)&pcl_dbf, sizeof(pcl_dbf));
		sio_ack(devno, cunit, 'C');
		com_write((uchar *)&pcl_dbf, sizeof(pcl_dbf));
		com_write(&sck, 1);
		goto exit;
	}

	if (fno == 0x07)	/* FCLOSE */
	{
		uchar fpmode;
		time_t mtime;
		char pathname[1024];

		if (ccom == 'R')
		{
			sio_ack(devno, cunit, 'A');	/* ack the command */
			device[devno][cunit].status.err = 176;
			printf("bad exec\n");
			goto complete;
		}

		if ((handle > 15) || (iodesc[handle].fps.file == NULL))
		{
			printf("bad handle %d\n", handle);
			device[devno][cunit].status.err = 134;	/* bad file handle */
			goto complete;
		}

		printf("handle %d\n", handle);

		device[devno][cunit].status.err = 1;

		fpmode = iodesc[handle].fpmode;
		mtime = iodesc[handle].fpstat.st_mtime;
# if 0
		printf("FCLOSE: mtime $%08x\n", mtime);
# endif
		strcpy(pathname, iodesc[handle].pathname);

		fps_close(handle);	/* this clears out iodesc[handle] */

		if (mtime && (fpmode & 0x08))
		{
			struct timeval tv[2];

			tv[0].tv_usec = 0;
			tv[0].tv_sec = mtime;
			tv[1].tv_usec = 0;
			tv[1].tv_sec = mtime;

# if 0
			printf("FCLOSE: setting timestamp in '%s'\n", pathname);
# endif

			(void)utimes(pathname, (void *)&tv);
		}
		goto complete;
	}

	if (fno == 0x08)	/* INIT */
	{
		if (ccom == 'R')
		{
			sio_ack(devno, cunit, 'A');	/* ack the command */
			device[devno][cunit].status.err = 176;
			printf("bad exec\n");
			goto complete;
		}

		do_pclink_init(0);

		device[devno][cunit].parbuf.handle = 0xff;
		device[devno][cunit].status.none = PCLSIO;
		device[devno][cunit].status.err = 1;
		goto complete;
	}

	if ((fno == 0x09) || (fno == 0x0a))	/* FOPEN/FFIRST */
	{
		if (ccom == 'P')
		{
			printf("mode: $%02x, atr1: $%02x, atr2: $%02x, path: '%s', name: '%s'\n", \
				device[devno][cunit].parbuf.fmode, device[devno][cunit].parbuf.fatr1, \
				device[devno][cunit].parbuf.fatr2, device[devno][cunit].parbuf.path, \
				device[devno][cunit].parbuf.name);
# if 0
			printf("date: %02d-%02d-%02d time: %02d:%02d:%02d\n", \
				device[devno][cunit].parbuf.f1, device[devno][cunit].parbuf.f2, \
				device[devno][cunit].parbuf.f3, device[devno][cunit].parbuf.f4, \
				device[devno][cunit].parbuf.f5, device[devno][cunit].parbuf.f6);
# endif

			device[devno][cunit].status.err = 1;

			if (fno == 0x0a)
				device[devno][cunit].parbuf.fmode |= 0x10;
			goto complete;
		}
		else	/* ccom not 'P', execution stage */
		{
			DIR *dh;
			uchar i;
			long sl;
			struct stat tempstat;
			char newpath[1024], raw_name[12];

			if ((ccom == 'R') && (old_ccom == 'R'))
			{
				sio_ack(devno, cunit, 'N');
				printf("serial communication error, abort\n");
				return;
			}

			sio_ack(devno, cunit, 'A');	/* ack the command */

			bzero(raw_name, sizeof(raw_name));
			memcpy(raw_name, device[devno][cunit].parbuf.name, 8+3);

			if (((device[devno][cunit].parbuf.fmode & 0x0c) == 0) || \
				((device[devno][cunit].parbuf.fmode & 0x18) == 0x18)) 
			{
				printf("unsupported fmode ($%02x)\n", device[devno][cunit].parbuf.fmode);
				device[devno][cunit].status.err = 146;
				goto complete_fopen;
			}

			create_user_path(devno, cunit, newpath);

			if (!validate_user_path(device[devno][cunit].dirname, newpath))
			{
				printf("invalid path '%s'\n", newpath);
				device[devno][cunit].status.err = 150;
				goto complete_fopen;
			}

			printf("local path '%s'\n", newpath);

			for (i = 0; i < 16; i++)
			{
# if 0
				printf("FOPEN: find handle: %d is $%08lx\n", i, (ulong)iodesc[i].fps.file);
# endif
				if (iodesc[i].fps.file == NULL)
# if 1
					break;
# else
				{
					printf("FOPEN: find handle: found %d\n", i);
					break;
				}
# endif
			}
			if (i > 15)
			{
				printf("FOPEN: too many channels open\n");
				device[devno][cunit].status.err = 161;
				goto complete_fopen;
			}

			if (stat(newpath, &tempstat) < 0)
			{
				printf("FOPEN: cannot stat '%s'\n", newpath);
				device[devno][cunit].status.err = 150;
				goto complete_fopen;
			}

			dh = opendir(newpath);

			if (device[devno][cunit].parbuf.fmode & 0x10)
			{
				iodesc[i].fps.dir = dh;
				memcpy(&sb, &tempstat, sizeof(sb));
			}
			else
			{
				while ((dp = readdir(dh)) != NULL)
				{
					if (check_dos_name(newpath, dp, &sb))
						continue;
		
					/* convert 8+3 to NNNNNNNNXXX */
					ugefina(dp->d_name, raw_name);

					/* match */
					if (match_dos_names(raw_name, \
						(char *)device[devno][cunit].parbuf.name, \
							device[devno][cunit].parbuf.fatr1, &sb) == 0)
						break;
				}

				sl = strlen(newpath);
				if (sl && (newpath[sl-1] != '/'))
					strcat(newpath, "/");

				if (dp)
				{
					strcat(newpath, dp->d_name);
					ugefina(dp->d_name, raw_name);
					if ((device[devno][cunit].parbuf.fmode & 0x0c) == 0x08)
						sb.st_mtime = timestamp2mtime(&device[devno][cunit].parbuf.f1);
				}
				else
				{
					if ((device[devno][cunit].parbuf.fmode & 0x0c) == 0x04)
					{
						printf("FOPEN: file not found\n");
						device[devno][cunit].status.err = 170;
						closedir(dh);
						dp = NULL;
						goto complete_fopen;
					}
					else
					{
						char name83[12];

						printf("FOPEN: creating file\n");

						uexpand(device[devno][cunit].parbuf.name, name83);

						if (validate_dos_name(name83))
						{
							printf("FOPEN: bad filename '%s'\n", name83);
							device[devno][cunit].status.err = 165; /* bad filename */
							goto complete_fopen;
						}

						strcat(newpath, name83);
						ugefina(name83, raw_name);

						bzero(&sb, sizeof(struct stat));
						sb.st_mode = S_IFREG|S_IRUSR|S_IWUSR;

						sb.st_mtime = timestamp2mtime(&device[devno][cunit].parbuf.f1);
					}
				}

				printf("FOPEN: full local path '%s'\n", newpath);

				if (stat(newpath, &tempstat) < 0)
				{
					if ((device[devno][cunit].parbuf.fmode & 0x0c) == 0x04)
					{
						printf("FOPEN: cannot stat '%s'\n", newpath);
						device[devno][cunit].status.err = 170;
						goto complete_fopen;
					}
				}
				else
				{
					if (device[devno][cunit].parbuf.fmode & 0x08)
					{
						if ((tempstat.st_mode & S_IWUSR) == 0)
						{
							printf("FOPEN: '%s' is read-only\n", newpath);
							device[devno][cunit].status.err = 151;
							goto complete_fopen;
						}
					}
# if 0
					if ((device[devno][cunit].parbuf.fmode & 0x0d) == 0x08)
					{
						if (!S_ISDIR(tempstat.st_mode))
						{
							printf("FOPEN: delete '%s'\n", newpath);
							if (unlink(newpath))
							{
								printf("FOPEN: cannot delete '%s'\n", newpath);
								device[devno][cunit].status.err = 255;
							}
						}
					}
# endif
				}

				if ((device[devno][cunit].parbuf.fmode & 0x0d) == 0x04)
					iodesc[i].fps.file = fopen(newpath, "r");
				else if ((device[devno][cunit].parbuf.fmode & 0x0d) == 0x08)
				{
					iodesc[i].fps.file = fopen(newpath, "w");
					if (iodesc[i].fps.file)
						sb.st_size = 0;
				}
				else if ((device[devno][cunit].parbuf.fmode & 0x0d) == 0x09)
				{
					iodesc[i].fps.file = fopen(newpath, "r+");
					if (iodesc[i].fps.file)
						fseek(iodesc[i].fps.file, sb.st_size, SEEK_SET);
				}
				else if ((device[devno][cunit].parbuf.fmode & 0x0d) == 0x0c)
					iodesc[i].fps.file = fopen(newpath, "r+");

				closedir(dh);
				dp = NULL;
			}

			if (iodesc[i].fps.file == NULL)
			{
				printf("FOPEN: cannot open '%s', %s (%d)\n", newpath, strerror(errno), errno);
				if (device[devno][cunit].parbuf.fmode & 0x04)
					device[devno][cunit].status.err = 170;
				else
					device[devno][cunit].status.err = 151;
				goto complete_fopen;
			}

# if 0
			printf("FOPEN: handle %d is $%08lx\n", i, (ulong)iodesc[i].fps.file);
# endif
			handle = device[devno][cunit].parbuf.handle = i;

			iodesc[handle].devno = devno;
			iodesc[handle].cunit = cunit;
			iodesc[handle].fpmode = device[devno][cunit].parbuf.fmode;
			iodesc[handle].fatr1 = device[devno][cunit].parbuf.fatr1;
			iodesc[handle].fatr2 = device[devno][cunit].parbuf.fatr2;
			iodesc[handle].t1 = device[devno][cunit].parbuf.f1;
			iodesc[handle].t2 = device[devno][cunit].parbuf.f2;
			iodesc[handle].t3 = device[devno][cunit].parbuf.f3;
			iodesc[handle].d1 = device[devno][cunit].parbuf.f4;
			iodesc[handle].d2 = device[devno][cunit].parbuf.f5;
			iodesc[handle].d3 = device[devno][cunit].parbuf.f6;
			iodesc[handle].fppos = 0L;
			strcpy(iodesc[handle].pathname, newpath);
			memcpy((void *)&iodesc[handle].fpstat, (void *)&sb, sizeof(struct stat));
			if (iodesc[handle].fpmode & 0x10)
				memcpy(iodesc[handle].fpname, device[devno][cunit].parbuf.name, sizeof(iodesc[i].fpname));
			else
				memcpy(iodesc[handle].fpname, raw_name, sizeof(iodesc[handle].fpname));

			iodesc[handle].fpstat.st_size = get_file_len(handle);

			if ((iodesc[handle].fpmode & 0x1d) == 0x09)
				iodesc[handle].fppos = iodesc[handle].fpstat.st_size;

			bzero(pcl_dbf.dirbuf, sizeof(pcl_dbf.dirbuf));

			if ((handle > 15) || (iodesc[handle].fps.file == NULL))
			{
				printf("FOPEN: bad handle %d\n", handle);
				device[devno][cunit].status.err = 134;	/* bad file handle */
				pcl_dbf.handle = 134;
			}
			else
			{
				pcl_dbf.handle = handle;

				unix_time_2_sdx(&iodesc[handle].fpstat.st_mtime, ob);

# if 0
				printf("FOPEN: time %02d-%02d-%02d %02d:%02d.%02d\n", ob[0], ob[1], ob[2], ob[3], ob[4], ob[5]);
# endif

				printf("FOPEN: %s handle %d\n", (iodesc[handle].fpmode & 0x08) ? "write" : "read", handle);

				bzero(pcl_dbf.dirbuf, sizeof(pcl_dbf.dirbuf));

				if (iodesc[handle].fpmode & 0x10)
				{
					int eof_sig;

					iodesc[handle].dir_cache = cache_dir(handle);
					iodesc[handle].fppos += dir_read(pcl_dbf.dirbuf, sizeof(pcl_dbf.dirbuf), handle, &eof_sig);

					if (eof_sig)
					{
						printf("FOPEN: dir EOF?\n");
						device[devno][cunit].status.err = 136;
					}
					else if (iodesc[handle].fppos == iodesc[handle].fpstat.st_size)
							device[devno][cunit].status.err = 3;
				}
				else
				{
					int x;
					ulong dlen = iodesc[handle].fpstat.st_size;

					memset(pcl_dbf.dirbuf+6, 0x20, 11);
					pcl_dbf.dirbuf[3] = (uchar)(dlen & 0x000000ffL);
					pcl_dbf.dirbuf[4] = (uchar)((dlen & 0x0000ff00L) >> 8);
					pcl_dbf.dirbuf[5] = (uchar)((dlen & 0x00ff0000L) >> 16);
					memcpy(pcl_dbf.dirbuf+17, ob, 6);

					pcl_dbf.dirbuf[0] = 0x08;

					if ((iodesc[handle].fpstat.st_mode & S_IWUSR) == 0)
						pcl_dbf.dirbuf[0] |= 0x01;	/* protected */
					if (S_ISDIR(iodesc[handle].fpstat.st_mode))
						pcl_dbf.dirbuf[0] |= 0x20;	/* directory */

					x = 0;
					while (iodesc[handle].fpname[x] && (x < 11))
					{
						pcl_dbf.dirbuf[6+x] = iodesc[handle].fpname[x];
						x++;
					}
				}

				printf("FOPEN: send $%02x $%02x%02x $%02x%02x%02x %c%c%c%c%c%c%c%c%c%c%c %02d-%02d-%02d %02d:%02d:%02d\n", \
				pcl_dbf.dirbuf[0],
				pcl_dbf.dirbuf[2], pcl_dbf.dirbuf[1],
				pcl_dbf.dirbuf[5], pcl_dbf.dirbuf[4], pcl_dbf.dirbuf[3],
				pcl_dbf.dirbuf[6], pcl_dbf.dirbuf[7], pcl_dbf.dirbuf[8], pcl_dbf.dirbuf[9],
				pcl_dbf.dirbuf[10], pcl_dbf.dirbuf[11], pcl_dbf.dirbuf[12], pcl_dbf.dirbuf[13],
				pcl_dbf.dirbuf[14], pcl_dbf.dirbuf[15], pcl_dbf.dirbuf[16],
				pcl_dbf.dirbuf[17], pcl_dbf.dirbuf[18], pcl_dbf.dirbuf[19],
				pcl_dbf.dirbuf[20], pcl_dbf.dirbuf[21], pcl_dbf.dirbuf[22]);
			}

complete_fopen:
			sck = calc_checksum((void *)&pcl_dbf, sizeof(pcl_dbf));
			sio_ack(devno, cunit, 'C');
			com_write((uchar *)&pcl_dbf, sizeof(pcl_dbf));
			com_write(&sck, 1);
			goto exit;
		}
	}

	if (fno == 0x0b)	/* RENAME/RENDIR */
	{
		char newpath[1024];
		DIR *renamedir;
		ulong fcnt = 0;

		if (ccom == 'R')
		{
			sio_ack(devno, cunit, 'A');	/* ack the command */
			device[devno][cunit].status.err = 176;
			printf("bad exec\n");
			goto complete;
		}

		create_user_path(devno, cunit, newpath);

		if (!validate_user_path(device[devno][cunit].dirname, newpath))
		{
			printf("invalid path '%s'\n", newpath);
			device[devno][cunit].status.err = 150;
			goto complete;
		}

		renamedir = opendir(newpath);

		if (renamedir == NULL)
		{
			printf("cannot open dir '%s'\n", newpath);
			device[devno][cunit].status.err = 255;
			goto complete;
		}

		printf("local path '%s', fatr1 $%02x\n", newpath, \
			device[devno][cunit].parbuf.fatr1 | RA_NO_PROTECT);

		device[devno][cunit].status.err = 1;

		while ((dp = readdir(renamedir)) != NULL)
		{
			char raw_name[12];
 
			if (check_dos_name(newpath, dp, &sb))
				continue;

			/* convert 8+3 to NNNNNNNNXXX */
			ugefina(dp->d_name, raw_name);

			/* match */
			if (match_dos_names(raw_name, (char *)device[devno][cunit].parbuf.name, \
				device[devno][cunit].parbuf.fatr1 | RA_NO_PROTECT, &sb) == 0)
			{
				char xpath[1024], xpath2[1024], newname[16];
				uchar names[12];
				struct stat dummy;
				ushort x;

				fcnt++;

				strcpy(xpath, newpath);
				strcat(xpath, "/");
				strcat(xpath, dp->d_name);

				memcpy(names, device[devno][cunit].parbuf.names, 12);

				for (x = 0; x < 12; x++)
				{
					if (names[x] == '?')
						names[x] = raw_name[x];
				}

				uexpand(names, newname);

				strcpy(xpath2, newpath);
				strcat(xpath2, "/");
				strcat(xpath2, newname);

				printf("RENAME: renaming '%s' -> '%s'\n", dp->d_name, newname);

				if (stat(xpath2, &dummy) == 0)
				{
					printf("RENAME: '%s' already exists\n", xpath2);
					device[devno][cunit].status.err = 151;
					break;
				}

				if (rename(xpath, xpath2))
				{
					printf("RENAME: %s\n", strerror(errno));
					device[devno][cunit].status.err = 255;
				}
			}
		}

		closedir(renamedir);

		if ((fcnt == 0) && (device[devno][cunit].status.err == 1))
			device[devno][cunit].status.err = 170;
		goto complete;
	}

	if (fno == 0x0c)	/* REMOVE */
	{
		char newpath[1024];
		DIR *deldir;
		ulong delcnt = 0;

		if (ccom == 'R')
		{
			sio_ack(devno, cunit, 'A');	/* ack the command */
			device[devno][cunit].status.err = 176;
			printf("bad exec\n");
			goto complete;
		}

		create_user_path(devno, cunit, newpath);

		if (!validate_user_path(device[devno][cunit].dirname, newpath))
		{
			printf("invalid path '%s'\n", newpath);
			device[devno][cunit].status.err = 150;
			goto complete;
		}

		printf("local path '%s'\n", newpath);

		deldir = opendir(newpath);

		if (deldir == NULL)
		{
			printf("cannot open dir '%s'\n", newpath);
			device[devno][cunit].status.err = 255;
			goto complete;
		}

		device[devno][cunit].status.err = 1;

		while ((dp = readdir(deldir)) != NULL)
		{
			char raw_name[12];
 
			if (check_dos_name(newpath, dp, &sb))
				continue;

			/* convert 8+3 to NNNNNNNNXXX */
			ugefina(dp->d_name, raw_name);

			/* match */
			if (match_dos_names(raw_name, (char *)device[devno][cunit].parbuf.name, \
				RA_NO_PROTECT | RA_NO_SUBDIR | RA_NO_HIDDEN, &sb) == 0)
			{
				char xpath[1024];

				strcpy(xpath, newpath);
				strcat(xpath, "/");
				strcat(xpath, dp->d_name);

				if (!S_ISDIR(sb.st_mode))
				{				
					printf("REMOVE: delete '%s'\n", xpath);
					if (unlink(xpath))
					{
						printf("REMOVE: cannot delete '%s'\n", xpath);
						device[devno][cunit].status.err = 255;
					}
					delcnt++;
				}
			}
		}
		closedir(deldir);
		if (delcnt == 0)
			device[devno][cunit].status.err = 170;
		goto complete;
	}

	if (fno == 0x0d)	/* CHMOD */
	{
		char newpath[1024];
		DIR *chmdir;
		ulong fcnt = 0;
		uchar fatr2 = device[devno][cunit].parbuf.fatr2;

		if (ccom == 'R')
		{
			sio_ack(devno, cunit, 'A');	/* ack the command */
			device[devno][cunit].status.err = 176;
			printf("bad exec\n");
			goto complete;
		}

		if (fatr2 & (SA_SUBDIR | SA_UNSUBDIR))
		{
			printf("illegal fatr2 $%02x\n", fatr2);
			device[devno][cunit].status.err = 146;
			goto complete;
		}

		create_user_path(devno, cunit, newpath);

		if (!validate_user_path(device[devno][cunit].dirname, newpath))
		{
			printf("invalid path '%s'\n", newpath);
			device[devno][cunit].status.err = 150;
			goto complete;
		}

		printf("local path '%s', fatr1 $%02x fatr2 $%02x\n", newpath, \
				device[devno][cunit].parbuf.fatr1, fatr2);

		chmdir = opendir(newpath);

		if (chmdir == NULL)
		{
			printf("CHMOD: cannot open dir '%s'\n", newpath);
			device[devno][cunit].status.err = 255;
			goto complete;
		}


		device[devno][cunit].status.err = 1;

		while ((dp = readdir(chmdir)) != NULL)
		{
			char raw_name[12];
 
			if (check_dos_name(newpath, dp, &sb))
				continue;

			/* convert 8+3 to NNNNNNNNXXX */
			ugefina(dp->d_name, raw_name);

			/* match */
			if (match_dos_names(raw_name, (char *)device[devno][cunit].parbuf.name, \
				device[devno][cunit].parbuf.fatr1, &sb) == 0)
			{
				char xpath[1024];
				mode_t newmode = sb.st_mode;

				strcpy(xpath, newpath);
				strcat(xpath, "/");
				strcat(xpath, dp->d_name);
				printf("CHMOD: change atrs in '%s'\n", xpath);

				/* On Unix, ignore Hidden and Archive bits */
				if (fatr2 & SA_UNPROTECT)
					newmode |= S_IWUSR;
				if (fatr2 & SA_PROTECT)
					newmode &= ~S_IWUSR;
				if (chmod(xpath, newmode))
				{
					printf("CHMOD: failed on '%s'\n", xpath);
					device[devno][cunit].status.err |= 255;
				}
				fcnt++;
			}
		}
		closedir(chmdir);
		if (fcnt == 0)
			device[devno][cunit].status.err = 170;
		goto complete;
	}

	if (fno == 0x0e)	/* MKDIR - warning, fatr2 is bogus */
	{
		char newpath[1024], fname[12];
		uchar dt[6];
		struct stat dummy;

		if (ccom == 'R')
		{
			sio_ack(devno, cunit, 'A');	/* ack the command */
			device[devno][cunit].status.err = 176;
			printf("bad exec\n");
			goto complete;
		}

		create_user_path(devno, cunit, newpath);

		if (!validate_user_path(device[devno][cunit].dirname, newpath))
		{
			printf("invalid path '%s'\n", newpath);
			device[devno][cunit].status.err = 150;
			goto complete;
		}

		uexpand(device[devno][cunit].parbuf.name, fname);

		if (validate_dos_name(fname))
		{
			printf("bad dir name '%s'\n", fname);
			device[devno][cunit].status.err = 165;
			goto complete;
		}

		strcat(newpath, "/");
		strcat(newpath, fname);

		memcpy(dt, &device[devno][cunit].parbuf.f1, sizeof(dt));

		printf("making dir '%s', time %2d-%02d-%02d %2d:%02d:%02d\n", newpath, \
			dt[0], dt[1], dt[2], dt[3], dt[4], dt[5]);

		if (stat(newpath, &dummy) == 0)
		{
			printf("MKDIR: '%s' already exists\n", newpath);
			device[devno][cunit].status.err = 151;
			goto complete;
		}

		if (mkdir(newpath, S_IRWXU|S_IRWXG|S_IRWXO))
		{
			printf("MKDIR: cannot make dir '%s'\n", newpath);
			device[devno][cunit].status.err = 255;
		}
		else
		{
			struct timeval tv[2];
			time_t mtime = timestamp2mtime(dt);

			device[devno][cunit].status.err = 1;

			if (mtime)
			{
				tv[0].tv_usec = 0;
				tv[0].tv_sec = mtime;
				tv[1].tv_usec = 0;
				tv[1].tv_sec = mtime;

# if 0
				printf("MKDIR: setting timestamp in '%s'\n", newpath);
# endif

				(void)utimes(newpath, (void *)&tv);
			}
		}
		goto complete;
	}

	if (fno == 0x0f)	/* RMDIR */
	{
		char newpath[1024], fname[12];

		if (ccom == 'R')
		{
			sio_ack(devno, cunit, 'A');	/* ack the command */
			device[devno][cunit].status.err = 176;
			printf("bad exec\n");
			goto complete;
		}

		create_user_path(devno, cunit, newpath);

		if (!validate_user_path(device[devno][cunit].dirname, newpath))
		{
			printf("invalid path '%s'\n", newpath);
			device[devno][cunit].status.err = 150;
			goto complete;
		}

		uexpand(device[devno][cunit].parbuf.name, fname);

		if (validate_dos_name(fname))
		{
			printf("bad dir name '%s'\n", fname);
			device[devno][cunit].status.err = 165;
			goto complete;
		}

		strcat(newpath, "/");
		strcat(newpath, fname);

		if (stat(newpath, &sb) < 0)
		{
			printf("cannot stat '%s'\n", newpath);
			device[devno][cunit].status.err = 170;
			goto complete;
		}

		if (sb.st_uid != our_uid)
		{
			printf("'%s' wrong uid\n", newpath);
			device[devno][cunit].status.err = 170;
			goto complete;
		}

		if (!S_ISDIR(sb.st_mode))
		{
			printf("'%s' is not a directory\n", newpath);
			device[devno][cunit].status.err = 170;
			goto complete;
		}

		if ((sb.st_mode & S_IWUSR) == 0)
		{
			printf("dir '%s' is write-protected\n", newpath);
			device[devno][cunit].status.err = 170;
			goto complete;
		}

		printf("delete dir '%s'\n", newpath);

		device[devno][cunit].status.err = 1;

		if (rmdir(newpath))
		{
			printf("RMDIR: cannot del '%s', %s (%d)\n", newpath, strerror(errno), errno);
			if (errno == ENOTEMPTY)
				device[devno][cunit].status.err = 167;
			else
				device[devno][cunit].status.err = 255;
		}
		goto complete;
	}

	if (fno == 0x10)	/* CHDIR */
	{
		ulong i;
		char newpath[1024], newwd[1024], oldwd[1024];

		if (ccom == 'R')
		{
			sio_ack(devno, cunit, 'A');	/* ack the command */
			device[devno][cunit].status.err = 176;
			printf("bad exec\n");
			goto complete;
		}

//		printf("req. path '%s'\n", device[devno][cunit].parbuf.path);

		create_user_path(devno, cunit, newpath);

		if (!validate_user_path(device[devno][cunit].dirname, newpath))
		{
			printf("invalid path '%s'\n", newpath);
			device[devno][cunit].status.err = 150;
			goto complete;
		}

		(void)getcwd(oldwd, sizeof(oldwd));

		if (chdir(newpath))
		{
			printf("cannot access '%s', %s\n", newpath, strerror(errno));
			device[devno][cunit].status.err = 150;
			goto complete;
		}

		(void)getcwd(newwd, sizeof(newwd));

# if 0
		printf("newwd %s\n", newwd);
# endif
		/* validate_user_path() guarantees that .dirname is part of newwd */
		i = strlen(device[devno][cunit].dirname);
		strcpy((char *)device[devno][cunit].cwd, newwd + i);
		printf("new current dir '%s'\n", (char *)device[devno][cunit].cwd);

		device[devno][cunit].status.err = 1;

		(void)chdir(oldwd);

		goto complete;
	}

	if (fno == 0x11)	/* GETCWD */
	{
		int i;
		uchar tempcwd[65];

		device[devno][cunit].status.err = 1;

		if (ccom == 'P')
		{
			printf("device $%02x\n", cunit);
			goto complete;
		}

		sio_ack(devno, cunit, 'A');	/* ack the command */

		tempcwd[0] = 0;

		for (i = 0; device[devno][cunit].cwd[i] && (i < 64); i++)
		{
			uchar a;

			a = toupper(device[devno][cunit].cwd[i]);
			if (a == '/')
				a = '>';
			tempcwd[i] = a;
		}

		tempcwd[i] = 0;

		printf("send '%s'\n", tempcwd);

		sck = calc_checksum(tempcwd, sizeof(tempcwd)-1);
		sio_ack(devno, cunit, 'C');
		com_write(tempcwd, sizeof(tempcwd)-1);
		com_write(&sck, sizeof(sck));
		goto exit;
	}

	if (fno == 0x13)	/* DFREE */
	{
		FILE *vf;
		int x;
		uchar c = 0, volname[8];
		char lpath[1024];
		static uchar dfree[65] =
		{
			0x21,		/* data format version */
			0x00, 0x00,	/* main directory ptr */
			0xff, 0xff, 	/* total sectors */
			0xff, 0xff,	/* free sectors */
			0x00,		/* bitmap length */
			0x00, 0x00,	/* bitmap begin */
			0x00, 0x00,	/* filef */
			0x00, 0x00,	/* dirf */
			0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,	/* volume name */
			0x00,		/* number of tracks */
			0x01,		/* bytes per sector, encoded */
			0x80,		/* version number */
			0x00, 0x02,	/* real bps */
			0x00, 0x00,	/* fmapen */
			0x01,		/* sectors per cluster */
			0x00, 0x00,	/* nr seq and rnd */
			0x00, 0x00,	/* bootp */
			0x00,		/* lock */

			0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,
			0,0,0,0,0,
			0		/* CRC */
		};

		device[devno][cunit].status.err = 1;

		if (ccom == 'P')
		{
			printf("device $%02x\n", cunit);
			goto complete;
		}

		sio_ack(devno, cunit, 'A');	/* ack the command */

		memset(dfree + 0x0e, 0x020, 8);

		strcpy(lpath, (char *)device[devno][cunit].dirname);
		strcat(lpath, "/");
		strcat(lpath, DEVICE_LABEL);

		printf("reading '%s'\n", lpath);

		vf = fopen(lpath, "r");

		if (vf)
		{
			int r;
			uchar a;

			r = fread(volname, sizeof (uchar), 8, vf);

			fclose(vf);

			for (x = 0; x < r; x++)
			{
				a = volname[x];
				if (a == 0x9b)
					break;
				dfree[14+x] = a;
			}
		}

		x = 0;
		while (x < 8)
		{
			c |= dfree[14+x];
			x++;
		}

		if (c == 0x20)
		{
			memcpy(dfree + 14, "PCLink  ", 8);
			dfree[21] = cunit + 0x40;
		}

		printf("DFREE: send info (%d bytes)\n", (int)sizeof(dfree)-1);

		dfree[64] = calc_checksum(dfree, sizeof(dfree)-1);
		sio_ack(devno, cunit, 'C');
		com_write(dfree, sizeof(dfree));
		goto exit;
	}

	if (fno == 0x14)	/* CHVOL */
	{
		FILE *vf;
		ulong nl;
		char lpath[1024];

		device[devno][cunit].status.err = 1;

		if (ccom == 'R')
		{
			sio_ack(devno, cunit, 'A');	/* ack the command */
			device[devno][cunit].status.err = 176;
			printf("bad exec\n");
			goto complete;
		}

		nl = strlen((char *)device[devno][cunit].parbuf.name);

		if (nl == 0)
		{
			printf("invalid name\n");
			device[devno][cunit].status.err = 156;
			goto complete;
		}

		strcpy(lpath, device[devno][cunit].dirname);
		strcat(lpath, "/");
		strcat(lpath, DEVICE_LABEL);

		printf("writing '%s'\n", lpath);

		vf = fopen(lpath, "w");

		if (vf)
		{
			int x;
			uchar a;

			for (x = 0; x < 8; x++)
			{
				a = device[devno][cunit].parbuf.name[x];
				if (!a || (a == 0x9b))
					a = 0x20;
				(void)fwrite(&a, sizeof(uchar), 1, vf);
			}
			fclose(vf);
		}
		else
		{
			printf("CHVOL: %s\n", strerror(errno));
			device[devno][cunit].status.err = 255;
		}
		goto complete;
	}

	printf("fno $%02x: not implemented\n", fno);
	device[devno][cunit].status.err = 146;

complete:
	sio_ack(devno, cunit, 'C');

exit:
	old_ccom = ccom;

	return;
}

static ushort
check_desync(uchar *cmd, uchar cksum, uchar cka)
{
	uchar ccom, cid, cdev;

	/* According to the Atari docs, the drive should discard the command
	 * if there's CRC error in it, and take no action.
	 */
	if (cksum != cka)
	{
		if (log_flag)
			printf("Bad CRC in cmd: Atari = $%02x, PC = $%02x\n", cka, (uchar)cksum);
		return 1;
	}

	ccom = cmd[1];

	if (ccom < 0x21)
		return 1;

	cdev = cmd[0];

//	if ((ccom > 0x80) && ((cdev != 0x45) && (ccom != 0x93)))
//		return 1;

	cid = cmd[0] & 0xf0;

	if ((cid != 0x20) && (cid != 0x30) && (cid != 0x40) && (cid != 0x50) && (cdev != PCLSIO) && (cdev != 0x6f))
		return 1;

	return 0;
}

int
main(int argc, char **argv)
{
	struct termios com;
	struct pollfd instat, *insp = &instat;
	int d, ch, a, toff = 0, ascii_translation = 0;
	ulong i, counter = 0;
	char *pth, printer[1024], serial[128];	/* 128 bytes ought to be enough for everyone */

	for (d = 0; d < 8; d++)
		for (i = 0; i < 16; i++)
			device_reset(d, i);

	do_pclink_init(1);

	our_uid = getuid();

	pth = strstr(argv[0], "mkatr");

	if (pth && (pth[5] == 0))
	{
		int r = 0, full13force = 0;
		char newname[1024];
		int c, s, t, h, b;
		c = 9, s = 18, t = 40, h = 1, b = 128;
		while ((ch = getopt(argc, argv, "d:t:s:h:b:f?")) != -1)
		{
			switch (ch)
			{
				case 'f':
				{
					full13force = 1;
					break;
				}
				case 'd':
				{
					if (strcmp(optarg, "90k")==0 || strcmp(optarg, "ss/sd")==0)
					{
						c = 1;
					}
					else if (strcmp(optarg, "130k")==0 || strcmp(optarg, "ss/ed")==0)
					{
						c = 2;
					}
					else if (strcmp(optarg, "180k")==0 || strcmp(optarg, "ss/dd")==0)
					{
						c = 3;
					}
					else if (strcmp(optarg, "360k")==0 || strcmp(optarg, "ds/dd")==0)
					{
						c = 4;
					}
					else if (strcmp(optarg, "720k")==0 || strcmp(optarg, "ds/qd")==0)
					{
						c = 5;
					}
					else if (strcmp(optarg, "1440k")==0 || strcmp(optarg, "ds/hd")==0)
					{
						c = 6;
					}
					else if (strcmp(optarg, "16m")==0)
					{
						c = 7;
					}
					else if (strcmp(optarg, "32m")==0)
					{
						c = 8;
					}
					break;
				}
				case 't':
				{
					t = atoi(optarg);
					break;
				}
				case 's':
				{
					s = atoi(optarg);
					break;
				}
				case 'h':
				{
					h = atoi(optarg);
					break;
				}
				case 'b':
				{
					b = atoi(optarg);
					break;
				}
				case '?':
				default:
				{
					mkatr_usage();
					return -1;
				}
			}
		}

		for (i = 1; i < (ulong)argc; i++)
		{
			if (argv[i][0] == '-')
			{
				if (strlen(argv[i]) > 1)
				{
					if (strchr("f", argv[i][1]) != NULL)
						continue;
					else
						i++;
				}
			}
			else
			{
				strcpy(newname, argv[i]);
				r = 1;
			}
		}

		if (!r)
		{
			mkatr_usage();
			return -1;
		}

		r = make_atr(newname, c, t, s, h, b, full13force);

		if (r)
			printf("Error %d creating %s\n", r, newname);

		return r;
	}

	if (argc < 2)
	{
		sio2bsd_usage();
		return 0;
	}

	serial[0] = printer[0] = 0;

	if (serlock() < 0)
	{
		printf("Another SIO2BSD instance is already running.\n");
		return 1;
	}

	signal(SIGHUP, sig);	/* sigaction looks better :] */
	signal(SIGINT, sig);
	signal(SIGQUIT, sig);
	signal(SIGILL, sig);
	signal(SIGTRAP, sig);
	signal(SIGABRT, sig);
# ifndef NOT_FBSD
	signal(SIGEMT, sig);
# endif
	signal(SIGFPE, sig);
# if 0
	signal(SIGKILL, sig);
# endif
	signal(SIGBUS, sig);
	signal(SIGSEGV, sig);
	signal(SIGSYS, sig);
	signal(SIGPIPE, sig);
# if 0
	signal(SIGALRM, sig);
# endif
	signal(SIGTERM, sig);
# if 0
	signal(SIGURG, sig);
	signal(SIGSTOP, sig);
	signal(SIGTSTP, sig);
	signal(SIGCONT, sig);
	signal(SIGCHLD, sig);
	signal(SIGTTIN, sig);
	signal(SIGTTOU, sig);
	signal(SIGIO, sig);
# endif
	signal(SIGXCPU, sig);
	signal(SIGXFSZ, sig);
# if 0
	signal(SIGVTALRM, sig);
	signal(SIGPROF, sig);
	signal(SIGWINCH, sig);
	signal(SIGINFO, sig);
# endif
	signal(SIGUSR1, sig);
	signal(SIGUSR2, sig);
# ifndef NOT_FBSD
	signal(SIGTHR, sig);
# endif

# ifdef ULTRA
#   define OPTSTR "b:i:q:c:d:p:s:f:tmlu?8"
# else
#  define OPTSTR "d:p:s:f:tmlu?8"
# endif

	while ((ch = getopt(argc, argv, OPTSTR)) != -1)
	{
		switch (ch)
		{
			case '8':
			{
				block_percom = 1;

				break;
			}
			case 'm':
			{
				use_command = 1;

				break;
			}
			case 'l':
			{
# ifdef SIOTRACE			
				log_flag = 1;
# else
				printf("warning: -l option ignored\n"); 
# endif
				break;
			}
			case 'd':
			{
				bt_delay = atoi(optarg);
				break;
			}
			case 'p':
			{
				strcpy(printer, optarg);
				break;
			}
			case 's':
			{
				strcpy(serial, optarg);
				break;
			}
			case 't':
			{
				ascii_translation = 1;
				break;
			}
			case 'u':
			{
				upper_dir ^= 0x01;
				break;
			}
# ifdef ULTRA
			case 'b':
			{
				turbo_ix = atoi(optarg);
				break;
			}
			case 'i':
			{
				hs_ix = atoi(optarg);
				break;
			}
			
			case 'q':
			{
				if (strcmp(optarg, "pal")==0)
				{
					pokey_hz = POKEY_PAL_HZ;
				}
				else if (strcmp(optarg, "ntsc")==0)
				{
					pokey_hz = POKEY_NTSC_HZ;
				}
				else if (strcmp(optarg, "ntscf")==0)
				{
					pokey_hz = POKEY_NTSC_FREDDY_HZ;
				}
				else
				{
					pokey_hz = atof(optarg);
				}
				break;
			}
			
			case 'c':
			{
				pokey_const = atof(optarg);
				break;
			}
# endif
			case 'f':
			{
				break;
			}
		        
			case '?':
			default:
			{
				sio2bsd_usage();
				goto go_exit;
			}
		}
	}

	for (i = 1; i < (ulong)argc; i++)
	{
		if (argv[i][0] == '-')
		{
			if (strlen(argv[i]) > 1)
			{
				if (strchr("tmlu8", argv[i][1]) != NULL)
					continue;
				else
				{
					if (argv[i++][1] == 'f')
					{
						if ((drvcnt < 16) || (pclcnt < 16))
						{
							a = atr_open(argv[i], 1);
							if (a < 0)
								printf("Error %d opening %s\n", a, argv[i]);
						}
					}
				}
			}
			else
				drvcnt++;
		}
		else if ((drvcnt < 16) || (pclcnt < 16))
		{
			a = atr_open(argv[i], 0);
			if (a < 0)
				printf("Error %d opening %s\n", a, argv[i]);
		}
	}

	printf("PCLink directory filter allows %s case names\n", upper_dir ? "UPPER" : "lower");

	if (printer[0])
	{
		printer_fd = open(printer, O_WRONLY);
		if (printer_fd < 0)
		{
			printer_fd = creat(printer, S_IWUSR|S_IRUSR);
			if (printer_fd > -1)
				close(printer_fd);
			printer_fd = open(printer, O_WRONLY);
		}
		if (printer_fd > -1)
		{
			printf("Printer P1: %s\n", printer);
			if (ascii_translation)
				printf("ATASCII->ASCII translation enabled\n");
		}
	}

	if (serial[0] == 0)
		strcpy(serial, SERIAL);

	printf("Serial port: %s\n", serial);

# ifdef __linux__
# define SERFLAGS O_RDWR|O_NOCTTY
# else
# define SERFLAGS O_RDWR|O_NOCTTY|O_DIRECT
# endif

	serial_fd = open(serial, SERFLAGS);

	if (serial_fd < 0)
	{
		printf("%s (%d) opening %s\n", strerror(errno), errno, serial);
		goto go_exit;
	}

# ifdef ULTRA
	printf("POKEY quartz %f Hz and HS Index 0 constant %f is assumed\n", pokey_hz, pokey_const);
# endif

	siospeed[0].idx = 0;
	siospeed[0].baud = 0;
	siospeed[0].speed = 0;

# ifdef ULTRA
	siospeed[1].idx = 0x28;
	siospeed[2].idx = 0x10;
	siospeed[3].idx = 0x08;
	siospeed[4].idx = 0x05;
	siospeed[5].idx = 0x02;
	siospeed[6].idx = 0x01;
	siospeed[7].idx = 0x00;

# if B19200==19200
	for (d = 1; d < 8; d++)			/* FreeBSD */
		siospeed[d].speed = siospeed[d].baud = make_baudrate(siospeed[d].idx);
# else
	siospeed[1].speed = B19200;
	siospeed[1].baud = 19200;
	siospeed[2].speed = B38400;
	siospeed[2].baud = 38400;

# ifdef TIOCGSERIAL
	for (d = 3; d < 8; d++)			/* Linux */
	{
		siospeed[d].speed = B38400;
		siospeed[d].baud = make_baudrate(siospeed[d].idx);
	}
# else
	siospeed[3].speed = B57600;		/* Cygwin et al. */
	siospeed[3].baud = 57600;

	for (d = 4; d < 8; d++)
	{
		siospeed[d].speed = 0;
		siospeed[d].baud = 0;
	}
# endif
# endif /* B19200 */

# else	/* ULTRA */

	for (d = 1; d < 8; d++)
	{
		siospeed[d].idx = 0x28;
# if B19200==19200
		siospeed[b].speed = siospeed[d].baud = make_baudrate(0x28);
# else
		siospeed[d].speed = B19200;
		siospeed[d].baud = 19200;
# endif
	}
# endif	/* ULTRA */

	printf("Default speed: HSINDEX=%d (%d bits/sec.)\n", siospeed[1].idx, siospeed[1].baud);

# ifdef ULTRA
	siospeed[0].idx = hs_ix;
	siospeed[0].speed = siospeed[0].baud = make_baudrate(hs_ix);

	while ((turbo_ix > 8) || (siospeed[turbo_ix].baud == 0))
	{
		printf("Invalid turbo baudrate selected, resetting to defaults.\n");
		turbo_ix--;
	}

	printf("Default turbo: HSINDEX=%d (%d bits/sec.)\n", siospeed[ULTRA].idx, siospeed[ULTRA].baud);
	printf("User selected: HSINDEX=%d (%d bits/sec.)\n", siospeed[turbo_ix].idx, siospeed[turbo_ix].baud);
# endif

	insp->fd = serial_fd;
	insp->events = POLLIN;
	insp->revents = 0;

	tcgetattr(serial_fd, &dflt);
	tcgetattr(serial_fd, &com);

	cfmakeraw(&com);

	/* 1 start bit, 8 data bits, 1 stop bit, no parity, 19200 bps
	 */
	sio_setspeed(&com, 1);

	com.c_cflag &= ~CSIZE;
	com.c_cflag |= (CREAD|CLOCAL|CS8);		/* enable receiver, ignore modem status lines, 8 bits */
	com.c_cflag &= ~(CRTSCTS|PARENB|CSTOPB);	/* disable hw flow control, disable parity, use 1 stop bit */ 
	com.c_iflag |= (IGNBRK|IGNPAR);			/* enable ignore break condition and parity errors */
	com.c_iflag &= ~(IXON|IXOFF|IXANY);		/* disable I/O flow control */

# ifndef NOT_FBSD
	com.c_cflag &= ~CCAR_OFLOW;			/* ignore Carrier Detect line */
# endif

# ifdef COMMAND_LINE
	if (ioctl(serial_fd, TIOCMGET, &comstate) <  0)
		printf("warning: ioctl(TIOCMGET): %s\n", strerror(errno));
# endif

	setpriority(PRIO_PROCESS, 0, -20);

	if (tcsetattr(serial_fd, TCSAFLUSH, &com) == 0)
	{
# if 0
# if B19200=19200
		struct termios rcom;
# endif
# endif
		uchar cksum, cmd[5], cka, cdev, cunit, cid, ccom, devno;
		uchar sdxtime[8];		/* $ff + 6 bytes time + checksum */
		int caux1, caux2, sync_attempts;
		long sec;

		for (;;)
		{
			/* Read the command frame (4 bytes + CRC) */
			com_read(cmd, sizeof(cmd), COM_COMD);

			sync_attempts = 0;
retry:
			cka = cmd[4];

			cksum = calc_checksum(cmd, 4);

			if (check_desync(cmd, cksum, cka))
			{
				if (log_flag)
					printf("Desync: $%02x, $%02x, $%02x, $%02x Attempt: %d\n", cmd[0], cmd[1], cmd[2], cmd[3], sync_attempts);

				/* Apparent desynch */
				if (sync_attempts < 4)
				{
					sync_attempts++;
					for (i = 0; i < 4; i++)
						cmd[i] = cmd[i+1];
					if (poll(insp, 1, 0) > 0)
					{
						com_read(cmd+4, 1, COM_DATA);
						goto retry;
					}
				}
# ifdef ULTRA
				turbo(&com, turbo_on ? 0 : 1);
# endif
				continue;
			}

			/* Protocol scheme (according to JZ):
			 *
			 * Read data (the computer reads):
			 *
			 * ATARI --> command + CRC --> DRIVE
			 * ATARI <-- 'A'cknowledge <-- DRIVE
			 * ---------<execution time>--------
			 * ATARI <-- 'C'omplete    <-- DRIVE
			 * ATARI <-- sector + CRC  <-- DRIVE
			 * THE END
			 *
			 * Write data (the computer writes):
			 *
			 * ATARI --> command + CRC --> DRIVE
			 * ATARI <-- 'A'cknowledge <-- DRIVE
			 * ATARI --> sector + CRC  --> DRIVE
			 * ATARI <-- 'A'cknowledge <-- DRIVE
			 * ---------<execution time>--------
			 * ATARI <-- 'C'omplete    <-- DRIVE
			 * THE END
			 *
			 */

			cdev = cmd[0];
			ccom = cmd[1];
			caux1 = cmd[2];
			caux2 = cmd[3];

			cunit = cdev & 0x0f;
			cid = cdev & 0xf0;
			sec = caux1 + (caux2 << 8);

# ifdef SIOTRACE
			printf("%ld -> '%c': $%02x, $%02x, $%04lx ($%02x)", \
				counter, isprint(ccom&0x7f) ? ccom&0x7f : 0x20, \
			       		cdev, ccom, sec, cka);
# ifdef ULTRA
			if (turbo_on)
				printf(" US=%d\n", siospeed[turbo_ix].idx);
			else
				putchar('\n');
# else
			putchar('\n');
# endif

			counter++;
# endif

			devno = cid >> 4;

			/* PCLink has a priority over all other devices */
			if ((pclcnt > 1) && ((cdev == PCLSIO) || (cdev == 0x6f)))
			{
				cunit = caux2 & 0x0f;	/* PCLink ignores DUNIT */
				cdev = 0x6f;
				devno = cdev >> 4;

				/* cunit == 0 is init during warm reset */
				if ((cunit == 0) || device[devno][cunit].on)
				{
					switch (ccom)
					{
						case 'P':
						case 'R':
						{
							do_pclink(devno, ccom, caux1, caux2);
							break;
						}
						case 'S':	/* status */
						{	
							sio_send_status(devno, cunit);
# if 0
# if B19200==19200
							tcgetattr(serial_fd, &rcom);
							printf("I/O speed: %d/%d bits/sec.\n", rcom.c_ispeed, rcom.c_ospeed);
# endif
# endif
							break;
						}
# ifdef ULTRA
						case '?':	/* send hi-speed index */
						{
							wait_for_command_drop();
							if (siospeed[turbo_ix].idx != 40)
								sio_send_data_byte(devno, cunit, siospeed[turbo_ix].idx);
							else
								sio_ack(devno, cunit, 'N');
							break;
						}
# endif
						default:
						{
							sio_ack(devno, cunit, 'N');
							break;
						}
					}
				}
			}

			/* ordinary disk drive */
			else if ((devno == 0x03) && (device[devno][cunit].fd > -1))
			{
				/* Execute commands */
				switch (ccom)
				{
					/* standard commands */
					case 'R':	/* read sector */
					case 'V':
					{
						send_sector(devno, cunit, ccom, sec);
						break;
					}
					case 'P':	/* put sector */
					case 'W':	/* write sector */
					{
# ifdef SIOTRACE
						static uchar l_cunit = 0;
						static long l_sec = 0;
	
						if (cunit == l_cunit)
						{
							if (l_sec == sec)
								printf("SIO warning: dup write, sector $%04lx\n", sec);
						}
# endif						
						receive_sector(devno, cunit, sec);
# ifdef SIOTRACE
						l_cunit = cunit;
						l_sec = sec;
# endif
						break;
					}
					case 'S':	/* status */
					{
						if (toff == 0)
							get_sdx_time(sdxtime);
						device[devno][cunit].status.none = sdxtime[toff++];
						wait_for_command_drop();
						sio_send_status(devno, cunit);
						if (toff > 6) toff = 0;
# if 0
# if B19200==19200
						tcgetattr(serial_fd, &rcom);
						printf("I/O speed: %d/%d bits/sec.\n", rcom.c_ispeed, rcom.c_ospeed);
# endif
# endif
						break;
					}
					case 'N':	/* read percom */
					{
						if (block_percom == 0)
							send_percom(cunit);
						else
							sio_ack(devno, cunit, 'N');
						break;
					}
					case 'O':	/* write percom */
					{
						if (block_percom == 0)
							receive_percom(cunit);
						else
							sio_ack(devno, cunit, 'N');
						break;
					}
# ifdef ULTRA
					case '?':	/* send hi-speed index */
					{
						wait_for_command_drop();
						if (siospeed[turbo_ix].idx != 40)
							sio_send_data_byte(devno, cunit, siospeed[turbo_ix].idx);
						else
							sio_ack(devno, cunit, 'N');
						break;
					}
# endif
					case '"':	/* format 1050 */
					{
						if (device[devno][cunit].percom.trk == 1)
						{
							sio_ack(devno, cunit, 'N');
							continue;
						}
						setup_percom(cunit, percom_ed);
						/* fall through */
					}
					case '!':	/* format */
					{
						ushort spt = device[devno][cunit].percom.spt_hi * 256 + device[devno][cunit].percom.spt_lo;
						ushort bps = device[devno][cunit].percom.bps_hi * 256 + device[devno][cunit].percom.bps_lo;
						ushort trk = device[devno][cunit].percom.trk;

						if ((ccom == '!') && block_percom && (spt == 26) && (trk == 40) && (bps == 128) && \
							(device[devno][cunit].percom.flags & 0x04))
						{
							drive_setup(cunit, 720*128, 128);

							setup_status(cunit);

							report_percom(cunit);
						}

						format_atr(cunit, 0);
						break;
					}
					default:
					{
						sio_ack(devno, cunit, 'N');
						break;
					}
				}
			}
# if 1
			else if ((devno == 0x02) && (cunit == 1))
			{
				uchar devbuf[512];

				switch (ccom)
				{
					case 'S':
					{
						sio_send_status(devno, 0);
						break;
					}
# ifdef ULTRA
					case '?':	/* send hi-speed index */
					{
						sio_send_data_byte(devno, cunit, siospeed[turbo_ix].idx);
						break;
					}
# endif
					case 'n':	/* devinfo */
					{
						bzero(devbuf, sizeof(devbuf));

						sio_ack(devno, d, 'A');

						devbuf[2] = cunit;
						devbuf[7] = sizeof(devbuf) / 256;	/* sector size */
						devbuf[8] = 0xff;
						devbuf[9] = 0xff;			/* sector count */

						sprintf((char *)devbuf+16, "SIO2BSD unit %d", (int)cunit);

						cksum = calc_checksum(devbuf, sizeof(devbuf));

						sio_ack(devno, d, 'C');
						com_write(devbuf, sizeof(devbuf));
						com_write(&cksum, sizeof(cksum));

						break;
					}
					case 'R':	/* bogus read */
					{
						bzero(devbuf, sizeof(devbuf));

						sio_ack(devno, d, 'A');

						cksum = calc_checksum(devbuf, sizeof(devbuf));

						sio_ack(devno, d, 'C');
						com_write(devbuf, sizeof(devbuf));
						com_write(&cksum, sizeof(cksum));

						break;
					}
					default:
					{
						sio_ack(devno, cunit, 'N');
						break;
					}
				}
			}
# endif
			/* Printer port */
			else if ((printer_fd > -1) && (cdev == 0x40))
			{
				switch (ccom)
				{
					case 'S':
					{
						sio_send_status(devno, 0);
# if 0
# if B19200==19200
						tcgetattr(serial_fd, &rcom);
						printf("I/O speed: %d/%d bits/sec.\n", rcom.c_ispeed, rcom.c_ospeed);
# endif
# endif
						break;
					}
					case 'W':
					{
						switch (sec & 0x00ff)
						{
							case 0x44:
							{
								device[devno][0].bps = 0x14;
								break;
							}
							case 0x53:
							{
								device[devno][0].bps = 0x1d;
								break;
							}
							default:
							{
								device[devno][0].bps = 0x28;
								break;
							}
						}

						receive_sector(devno, cunit, 0);

						for (i = 0; i < device[devno][0].bps; i++)
						{
							if (inpbuf[i] == 0x9b)		/* EOL */
							{
								if (ascii_translation)
									inpbuf[i] = '\n';
								i++;
								break;
							}
							if (ascii_translation)
							{
								if (inpbuf[i] == 0x1c)
									inpbuf[i] = '.';
								else if (inpbuf[i] == 0x1d)
									inpbuf[i] = '.';
								else if (inpbuf[i] == 0x1e)
									inpbuf[i] = '\b';
								else if (inpbuf[i] == 0x1f)
									inpbuf[i] = '.';
								else if (inpbuf[i] == 0x7d)	/* clear screen */
									inpbuf[i] = '\f';
								else if (inpbuf[i] == 0x7e)	/* backspace */
									inpbuf[i] = '\b';
								else if (inpbuf[i] == 0x7f)	/* tabulate */
									inpbuf[i] = '\t';
								else if (inpbuf[i] == 0x9c)	/* delete line */
									inpbuf[i] = '.';
								else if (inpbuf[i] == 0x9d)	/* insert line */
									inpbuf[i] = '.';
								else if (inpbuf[i] == 0x9e)	/* clear tab */
									inpbuf[i] = '.';
								else if (inpbuf[i] == 0x9f)	/* set tab */
									inpbuf[i] = '.';
								else if (inpbuf[i] == 0xfd)	/* bell */
									inpbuf[i] = '\a';
								else if (inpbuf[i] == 0xfe)	/* delete char */
									inpbuf[i] = '.';
								else if (inpbuf[i] == 0xff)	/* insert char */
									inpbuf[i] = '.';
							}
						}
						if (write(printer_fd, inpbuf, i) != (long)i)
							sio_ack(devno, cunit, 'E');
						else
							sio_ack(devno, cunit, 'C');

						break;
					}
					default:
					{
						sio_ack(devno, cunit, 'N');
						break;
					}
				}
			}

			/* APE time protocol */
			else if (cdev == 0x45)
			{
				switch (ccom)
				{
					case 0x93:
					{
						if (sec == 0x000a0eeUL)
						{
							sio_ack(devno, cunit, 'A');
							get_sdx_time(sdxtime);
							sdxtime[7] = calc_checksum(sdxtime+1, 6);
							sio_ack(devno, cunit, 'C');
							com_write(sdxtime+1, 7);
# ifdef SIOTRACE
							if (log_flag)
								printf("<- APE TIME\n");
# endif
							break;
						}
						/* else fall through */
					}
					default:
					{
						sio_ack(devno, cunit, 'N');
						break;
					}
				}
			}
		}
	}
	else
		printf("tcsetattr(): %s (%d)\n", strerror(errno), errno);

go_exit:

	sig(0);		/* never returns */

# ifndef __GNUC__
	return 0;
# endif
}

/* EOF */

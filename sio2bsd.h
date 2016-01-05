/* SIO2BSD
 *
 * (c) 2005-11 KMK <drac030@krap.pl>
 *
 */

# if defined (NOT_FBSD)
#  include <endian.h>
# else
#  include <machine/endian.h>
# endif

# if BYTE_ORDER==BIG_ENDIAN
# define SSWAP(x) \
        ((((x) & 0xff00) >> 8) | \
         (((x) & 0x00ff) << 8))
# define LSWAP(x) \
	((((x) & 0xff000000) >> 24) | \
         (((x) & 0x00ff0000) >>  8) | \
         (((x) & 0x0000ff00) <<  8) | \
         (((x) & 0x000000ff) << 24))
# else
#  define SSWAP(x) (x)
#  define LSWAP(x) (x)
# endif

# ifdef ULTRA
#  if ULTRA<0x00 || ULTRA>0x06
#   error "Invalid turbo mode (0-6 allowed)"
#  endif
# endif

# if (PCLSIO&0xf0)>=0x80
#   error "Invalid PCLSIO value, suggest 0x6f"
# endif

# define SERLOCK "sio2bsd.lock"

# ifndef uchar
#  define uchar unsigned char
# endif

# ifndef ushort
#  define ushort unsigned short
# endif

# ifndef ulong
#  define ulong unsigned long
# endif

/* SIO speed */
typedef struct
{
        ushort  idx;		/* HS Index */
        speed_t baud;		/* corresponding bitrate */
	speed_t speed;		/* argument for cfsetspeed() */
} SIOSPEED;

/* Atari SIO status block */
typedef struct
{
	uchar stat;
	uchar err;
	uchar tmot;
	uchar none;
} STATUS;

/* Atari SIO PERCOM block */
typedef struct
{
        uchar trk;
        uchar step;
        uchar spt_hi;
        uchar spt_lo;
        uchar heads;
        uchar flags;
        uchar bps_hi;
        uchar bps_lo;
} PERCOM;

/* ATR file header */
typedef struct
{
	ushort sig;
	ushort wpars;
	ushort bps;
	uchar  hipars;
	ulong  crc;
	long  costam;
	uchar  prot;
} ATR;

typedef	struct			/* PCLink parameter buffer */
{
	uchar fno;		/* function number */
	uchar handle;		/* file handle */
	uchar f1,f2,f3,f4;	/* general-purpose bytes */
	uchar f5,f6;		/* more general-purpose bytes */
	uchar fmode;		/* fmode */
	uchar fatr1;		/* fatr1 */
	uchar fatr2;		/* fatr2 */
	uchar name[12];		/* name */
	uchar names[12];	/* names */
	uchar path[65];		/* path */
} PARBUF;

typedef struct
{
	ATR atr;		/* ATR file header */
	int fd;			/* ATR file handle */
	PERCOM percom;		/* the PERCOM data for the disk */
	STATUS status;		/* the 4-byte status block */
	ulong maxsec;		/* max. sector number */
	ushort bps;		/* number of bytes per sector */
	int full13;		/* if 1, the image has full-size bootsectors */
	int full13force;	/* if 1, the dd image will be full13 after reformat */
	
	int on;			/* PCLink mount flag */
	char dirname[1024];	/* PCLink root directory path */
	uchar cwd[65];		/* PCLink current working dir, relative to the above */
	PARBUF parbuf;		/* PCLink parameter buffer */
} DEVICE;

# ifdef __GNUC__
static void sig (int) __attribute__ ((__noreturn__));
# endif

/* EOF */

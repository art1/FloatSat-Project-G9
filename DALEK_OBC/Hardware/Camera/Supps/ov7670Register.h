
#define REG_GAIN        0x00    /* Gain lower 8 bits (rest in vref) */
#define REG_BLUE        0x01    /* blue gain */
#define REG_RED         0x02    /* red gain */
#define REG_VREF        0x03    /* Pieces of GAIN, VSTART, VSTOP */
#define REG_COM1        0x04    /* Control 1 */
#define COM1_CCIR656    0x40    /* CCIR656 enable */
#define REG_BAVE        0x05    /* U/B Average level */
#define REG_GbAVE       0x06    /* Y/Gb Average level */
#define REG_AECHH       0x07    /* AEC MS 5 bits */
#define REG_RAVE        0x08    /* V/R Average level */
#define REG_COM2        0x09    /* Control 2 */
#define COM2_SSLEEP     0x10    /* Soft sleep mode */
#define REG_PID         0x0a    /* Product ID MSB */
#define REG_VER         0x0b    /* Product ID LSB */
#define REG_COM3        0x0c    /* Control 3 */
#define COM3_SWAP       0x40    /* Byte swap */
#define COM3_SCALEEN    0x08    /* Enable scaling */
#define COM3_DCWEN      0x04    /* Enable downsamp/crop/window */
#define REG_COM4        0x0d    /* Control 4 */
#define REG_COM5        0x0e    /* All "reserved" */
#define REG_COM6        0x0f    /* Control 6 */
#define REG_AECH        0x10    /* More bits of AEC value */
#define REG_CLKRC       0x11    /* Clocl control */
#define CLK_EXT         0x40    /* Use external clock directly */
#define CLK_SCALE       0x3f    /* Mask for internal clock scale */
#define REG_COM7        0x12    /* Control 7 */
#define COM7_RESET      0x80    /* Register reset */
#define COM7_FMT_MASK   0x38
#define COM7_FMT_VGA    0x00
#define COM7_FMT_CIF    0x20    /* CIF format */
#define COM7_FMT_QVGA   0x10    /* QVGA format */
#define COM7_FMT_QCIF   0x08    /* QCIF format */
#define COM7_RGB        0x04    /* bits 0 and 2 - RGB format */
#define COM7_YUV        0x00    /* YUV */
#define COM7_BAYER      0x01    /* Bayer format */
#define COM7_PBAYER     0x05    /* "Processed bayer" */
#define REG_COM8        0x13    /* Control 8 */
#define COM8_FASTAEC    0x80    /* Enable fast AGC/AEC */
#define COM8_AECSTEP    0x40    /* Unlimited AEC step size */
#define COM8_BFILT      0x20    /* Band filter enable */
#define COM8_AGC        0x04    /* Auto gain enable */
#define COM8_AWB        0x02    /* White balance enable */
#define COM8_AEC        0x01    /* Auto exposure enable */
#define REG_COM9        0x14    /* Control 9  - gain ceiling */
#define REG_COM10       0x15    /* Control 10 */
#define COM10_HSYNC     0x40    /* HSYNC instead of HREF */
#define COM10_PCLK_HB   0x20    /* Suppress PCLK on horiz blank */
#define COM10_HREF_REV  0x08    /* Reverse HREF */
#define COM10_VS_LEAD   0x04    /* VSYNC on clock leading edge */
#define COM10_VS_NEG    0x02    /* VSYNC negative */
#define COM10_HS_NEG    0x01    /* HSYNC negative */
#define REG_HSTART      0x17    /* Horiz start high bits */
#define REG_HSTOP       0x18    /* Horiz stop high bits */
#define REG_VSTART      0x19    /* Vert start high bits */
#define REG_VSTOP       0x1a    /* Vert stop high bits */
#define REG_PSHFT       0x1b    /* Pixel delay after HREF */
#define REG_MIDH        0x1c    /* Manuf. ID high */
#define REG_MIDL        0x1d    /* Manuf. ID low */
#define REG_MVFP        0x1e    /* Mirror / vflip */
#define MVFP_MIRROR     0x20    /* Mirror image */
#define MVFP_FLIP       0x10    /* Vertical flip */
#define REG_AEW         0x24    /* AGC upper limit */
#define REG_AEB         0x25    /* AGC lower limit */
#define REG_VPT         0x26    /* AGC/AEC fast mode op region */
#define REG_HSYST       0x30    /* HSYNC rising edge delay */
#define REG_HSYEN       0x31    /* HSYNC falling edge delay */
#define REG_HREF        0x32    /* HREF pieces */
#define REG_TSLB        0x3a    /* lots of stuff */
#define TSLB_YLAST      0x04    /* UYVY or VYUY - see com13 */
#define REG_COM11       0x3b    /* Control 11 */
#define COM11_NIGHT     0x80    /* NIght mode enable */
#define COM11_NMFR      0x60    /* Two bit NM frame rate */
#define COM11_HZAUTO    0x10    /* Auto detect 50/60 Hz */
#define COM11_50HZ      0x08    /* Manual 50Hz select */
#define COM11_EXP       0x02
#define REG_COM12       0x3c    /* Control 12 */
#define COM12_HREF      0x80    /* HREF always */
#define REG_COM13       0x3d    /* Control 13 */
#define COM13_GAMMA     0x80    /* Gamma enable */
#define COM13_UVSAT     0x40    /* UV saturation auto adjustment */
#define COM13_UVSWAP    0x01    /* V before U - w/TSLB */
#define REG_COM14       0x3e    /* Control 14 */
#define COM14_DCWEN     0x10    /* DCW/PCLK-scale enable */
#define REG_EDGE        0x3f    /* Edge enhancement factor */
#define REG_COM15       0x40    /* Control 15 */
#define COM15_R10F0     0x00    /* Data range 10 to F0 */
#define COM15_R01FE     0x80    /*            01 to FE */
#define COM15_R00FF     0xc0    /*            00 to FF */
#define COM15_RGB565    0x10    /* RGB565 output */
#define COM15_RGB555    0x30    /* RGB555 output */
#define REG_COM16       0x41    /* Control 16 */
#define COM16_AWBGAIN   0x08    /* AWB gain enable */
#define REG_COM17       0x42    /* Control 17 */
#define COM17_AECWIN    0xc0    /* AEC window - must match COM4 */
#define COM17_CBAR      0x08    /* DSP Color bar */

#define REG_BRIGHT      0x55    /* Brightness */
#define REG_REG76       0x76    /* OV's name */
#define R76_BLKPCOR     0x80    /* Black pixel correction enable */
#define R76_WHTPCOR     0x40    /* White pixel correction enable */
#define REG_RGB444      0x8c    /* RGB 444 control */
#define R444_ENABLE     0x02    /* Turn on RGB444, overrides 5x5 */
#define R444_RGBX       0x01    /* Empty nibble at end */
#define REG_HAECC1      0x9f    /* Hist AEC/AGC control 1 */
#define REG_HAECC2      0xa0    /* Hist AEC/AGC control 2 */
#define REG_BD50MAX     0xa5    /* 50hz banding step limit */
#define REG_HAECC3      0xa6    /* Hist AEC/AGC control 3 */
#define REG_HAECC4      0xa7    /* Hist AEC/AGC control 4 */
#define REG_HAECC5      0xa8    /* Hist AEC/AGC control 5 */
#define REG_HAECC6      0xa9    /* Hist AEC/AGC control 6 */
#define REG_HAECC7      0xaa    /* Hist AEC/AGC control 7 */
#define REG_BD60MAX     0xab    /* 60hz banding step limit */

#define MTX1            0x4f    /* Matrix Coefficient 1 */
#define MTX2            0x50    /* Matrix Coefficient 2 */
#define MTX3            0x51    /* Matrix Coefficient 3 */
#define MTX4            0x52    /* Matrix Coefficient 4 */
#define MTX5            0x53    /* Matrix Coefficient 5 */
#define MTX6            0x54    /* Matrix Coefficient 6 */
#define REG_CONTRAS     0x56    /* Contrast control */
#define MTXS            0x58    /* Matrix Coefficient Sign */
#define AWBC7           0x59    /* AWB Control 7 */
#define AWBC8           0x5a    /* AWB Control 8 */
#define AWBC9           0x5b    /* AWB Control 9 */
#define AWBC10          0x5c    /* AWB Control 10 */
#define AWBC11          0x5d    /* AWB Control 11 */
#define AWBC12          0x5e    /* AWB Control 12 */
#define REG_GFIX        0x69    /* Fix gain control */
#define GGAIN           0x6a    /* G Channel AWB Gain */
#define DBLV            0x6b
#define AWBCTR3         0x6c    /* AWB Control 3 */
#define AWBCTR2         0x6d    /* AWB Control 2 */
#define AWBCTR1         0x6e    /* AWB Control 1 */
#define AWBCTR0         0x6f    /* AWB Control 0 */



//#define OV7660_CTL_GAIN				0x00
//#define OV7660_CTL_BLUE				0x01
//#define OV7660_CTL_RED				0x02
//#define OV7660_CTL_VREF				0x03
//#define OV7660_CTL_COM1				0x04
//#define OV7660_CTL_BAVE				0x05
//#define OV7660_CTL_GEAVE			0x06
//#define OV7660_CTL_AECHH			0x07
//#define OV7660_CTL_RAVE				0x08
//#define OV7660_CTL_COM2				0x09
//#define OV7660_CTL_PID				0x0a
//#define OV7660_CTL_VER				0x0b
//#define OV7660_CTL_COM3				0x0c
//#define OV7660_CTL_COM4				0x0d
//#define OV7660_CTL_COM5				0x0e
//#define OV7660_CTL_COM6				0x0f
//#define OV7660_CTL_AECH				0x10
//#define OV7660_CTL_CLKRC			0x11
//#define OV7660_CTL_COM7				0x12
//#define OV7660_CTL_COM8				0x13
//#define OV7660_CTL_COM9				0x14
//#define OV7660_CTL_COM10			0x15
///* RSVD 0x16 is Reserved */
//#define OV7660_CTL_HSTART			0x17
//#define OV7660_CTL_HSTOP			0x18
//#define OV7660_CTL_VSTRT			0x19
//#define OV7660_CTL_VSTOP			0x1a
//#define OV7660_CTL_PSHFT			0x1b
//#define OV7660_CTL_MIDH				0x1c
//#define OV7660_CTL_MIDL				0x1d
//#define OV7660_CTL_MVFP				0x1e
//#define OV7660_CTL_LAEC				0x1f
//#define OV7660_CTL_BOS				0x20
//#define OV7660_CTL_GBOS				0x21
//#define OV7660_CTL_GROS				0x22
//#define OV7660_CTL_ROS				0x23
//#define OV7660_CTL_AEW				0x24
//#define OV7660_CTL_AEB				0x25
//#define OV7660_CTL_VPT				0x26
//#define OV7660_CTL_BBIAS			0x27
//#define OV7660_CTL_GbBIAS			0x28
///* RSVD 0x29 is Reserved */
//#define OV7660_CTL_EXHCH			0x2a
//#define OV7660_CTL_EXHCL			0x2b
//#define OV7660_CTL_RBIAS			0x2c
//#define OV7660_CTL_ADVFL			0x2d
//#define OV7660_CTL_ADVFH			0x2e
//#define OV7660_CTL_YAVE				0x2f
//#define OV7660_CTL_HSYST			0x30
//#define OV7660_CTL_HSYEN			0x31
//#define OV7660_CTL_HREF				0x32
//#define OV7660_CTL_CHLF				0x33
//#define OV7660_CTL_ARBLM			0x34
///* RSVD 0x35 is Reserved */
///* RSVD 0x36 is Reserved */
//#define OV7660_CTL_ADC				0x37
//#define OV7660_CTL_ACOM				0x38
//#define OV7660_CTL_OFON				0x39
//#define OV7660_CTL_TSLB				0x3a
//#define OV7660_CTL_COM11			0x3b
//#define OV7660_CTL_COM12			0x3c
//#define OV7660_CTL_COM13			0x3d
//#define OV7660_CTL_COM14			0x3e
//#define OV7660_CTL_EDGE				0x3f
//#define OV7660_CTL_COM15			0x40
//#define OV7660_CTL_COM16			0x41
//#define OV7660_CTL_COM17			0x42
///* RSVD 0x43 is Reserved */
///* RSVD 0x44 is Reserved */
///* RSVD 0x45 is Reserved */
///* RSVD 0x46 is Reserved */
///* RSVD 0x47 is Reserved */
///* RSVD 0x48 is Reserved */
///* RSVD 0x49 is Reserved */
///* RSVD 0x4a is Reserved */
///* RSVD 0x4b is Reserved */
///* RSVD 0x4c is Reserved */
///* RSVD 0x4d is Reserved */
///* RSVD 0x4e is Reserved */
//#define OV7660_CTL_MTX1				0x4f
//#define OV7660_CTL_MTX2				0x50
//#define OV7660_CTL_MTX3				0x51
//#define OV7660_CTL_MTX4				0x52
//#define OV7660_CTL_MTX5				0x53
//#define OV7660_CTL_MTX6				0x54
//#define OV7660_CTL_MTX7				0x55
//#define OV7660_CTL_MTX8				0x56
//#define OV7660_CTL_MTX9				0x57
//#define OV7660_CTL_MTXS				0x58
///* RSVD 0x59 is Reserved */
///* RSVD 0x60 is Reserved */
///* RSVD 0x61 is Reserved */
//#define OV7660_CTL_LCC1				0x62
//#define OV7660_CTL_LCC2				0x63
//#define OV7660_CTL_LCC3				0x64
//#define OV7660_CTL_LCC4				0x65
//#define OV7660_CTL_LCC5				0x66
//#define OV7660_CTL_MANU				0x67
//#define OV7660_CTL_MANV				0x68
//#define OV7660_CTL_HV				0x69
//#define OV7660_CTL_GGAIN			0x6a
//#define OV7660_CTL_DBLV				0x6b
///* 6c-7b GSP */
///* 7c-8a GST */
///* RSVD 0x8b is Reserved */
///* RSVD 0x8c is Reserved */
///* RSVD 0x8d is Reserved */
///* RSVD 0x8e is Reserved */
///* RSVD 0x8f is Reserved */
///* RSVD 0x90 is Reserved */
///* RSVD 0x91 is Reserved */
//#define OV7660_CTL_DM_LNL			0x92
//#define OV7660_CTL_DM_LNH			0x93
///* RSVD 0x94 is Reserved */
///* RSVD 0x95 is Reserved */
///* RSVD 0x96 is Reserved */
///* RSVD 0x97 is Reserved */
///* RSVD 0x98 is Reserved */
///* RSVD 0x99 is Reserved */
///* RSVD 0x9a is Reserved */
///* RSVD 0x9b is Reserved */
///* RSVD 0x9c is Reserved */
//#define OV7660_CTL_BD50ST			0x9d
//#define OV7660_CTL_BD60ST			0x9e

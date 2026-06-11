typedef enum BMI270_REGS {
    CHIP_ID_ADDR            = 0x00,
    ERR_REG_ADDR            = 0x02,
    STATUS_ADDR             = 0x03,
    AUX_X_LSB_ADDR          = 0x04,
    ACC_X_LSB_ADDR          = 0x0C,
    GYR_X_LSB_ADDR          = 0x12,
    SENSORTIME_ADDR         = 0x18,
    EVENT_ADDR              = 0x1B,
    INT_STATUS_0_ADDR       = 0x1C,
    INT_STATUS_1_ADDR       = 0x1D,
    SC_OUT_0_ADDR           = 0x1E,
    SYNC_COMMAND_ADDR       = 0x1E,
    GYR_CAS_GPIO0_ADDR      = 0x1E,
    INTERNAL_STATUS_ADDR    = 0x21,
    TEMPERATURE_0_ADDR      = 0x22,
    FIFO_LENGTH_0_ADDR      = 0x24,
    FIFO_DATA_ADDR          = 0x26,

    FEAT_PAGE_ADDR          = 0x2F,
    FEATURES_REG_ADDR       = 0x30,

    ACC_CONF_ADDR           = 0x40,
    GYR_CONF_ADDR           = 0x42,
    AUX_CONF_ADDR           = 0x44,

    FIFO_DOWNS_ADDR         = 0x45,
    FIFO_WTM_0_ADDR         = 0x46,
    FIFO_WTM_1_ADDR         = 0x47,
    FIFO_CONFIG_0_ADDR      = 0x48,
    FIFO_CONFIG_1_ADDR      = 0x49,

    AUX_DEV_ID_ADDR         = 0x4B,
    AUX_IF_CONF_ADDR        = 0x4C,
    AUX_RD_ADDR             = 0x4D,
    AUX_WR_ADDR             = 0x4E,
    AUX_WR_DATA_ADDR        = 0x4F,

    INT1_IO_CTRL_ADDR       = 0x53,
    INT2_IO_CTRL_ADDR       = 0x54,

    INT_LATCH_ADDR          = 0x55,

    INT1_MAP_FEAT_ADDR      = 0x56,
    INT2_MAP_FEAT_ADDR      = 0x57,
    INT_MAP_DATA_ADDR       = 0x58,

    INIT_CTRL_ADDR          = 0x59,
    INIT_ADDR_0             = 0x5B,
    INIT_ADDR_1             = 0x5C,
    INIT_DATA_ADDR          = 0x5E,

    AUX_IF_TRIM             = 0x68,
    GYR_CRT_CONF_ADDR       = 0x69,

    NVM_CONF_ADDR           = 0x6A,

    IF_CONF_ADDR            = 0x6B,

    ACC_SELF_TEST_ADDR      = 0x6D,
    GYR_SELF_TEST_AXES_ADDR = 0x6E,
    SELF_TEST_MEMS_ADDR     = 0x6F,

    NV_CONF_ADDR            = 0x70,

    ACC_OFF_COMP_0_ADDR     = 0x71,
    GYR_OFF_COMP_3_ADDR     = 0x74,
    GYR_OFF_COMP_6_ADDR     = 0x77,
    GYR_USR_GAIN_0_ADDR     = 0x78,

    PWR_CONF_ADDR           = 0x7C,
    PWR_CTRL_ADDR           = 0x7D,

    CMD_REG_ADDR            = 0x7E
};

typedef enum BMI270_CMD {
    G_TRIGGER_CMD  = 0x02,
    USR_GAIN_CMD   = 0x03,
    NVM_PROG_CMD   = 0xA0,
    SOFT_RESET_CMD = 0xB6,
    FIFO_FLUSH_CMD = 0xB0
};

/*
Reg     Page0           Page1           Page2       Page3
0x30	SC_OUT_0_1	    Reserved	    NOMO_1	    SC_1
0x32	SC_OUT_2_3	    G_TRIG_1	    NOMO_2	    SC_2
0x34	ACT_OUT	        GEN_SET_1	    SIGMO_1	    SC_3
0x36	WR_GEST_OUT	    GYR_GAIN_UPD_1	Reserved	SC_4
0x38	GYR_GAIN_STATUS	GYR_GAIN_UPD_2	Reserved	SC_5
0x3A	Reserved	    GYR_GAIN_UPD_3	Reserved	SC_6
0x3C	GYR_CAS	        ANYMO_1	        Reserved	SC_7
0x3E	Reserved	    ANYMO_2	        SIGMO_2	    SC_8

Reg     Page4   Page5   Page6       Page7
0x30	SC_9	SC_17	SC_25	    WR_WAKEUP_1
0x32	SC_10	SC_18	SC_26	    WR_WAKEUP_2
0x34	SC_11	SC_19	SC_27	    WR_WAKEUP_3
0x36	SC_12	SC_20	WR_GEST_1	WR_WAKEUP_4
0x38	SC_13	SC_21	WR_GEST_2	WR_WAKEUP_5
0x3A	SC_14	SC_22	WR_GEST_3	WR_WAKEUP_6
0x3C	SC_15	SC_23	WR_GEST_4	WR_WAKEUP_7
0x3E	SC_16	SC_24	Reserved	Reserved
*/

typedef enum PAGENUM
{
    PAGE0= 0x000,
    PAGE1= 0x100,
    PAGE2= 0x200,
    PAGE3= 0x300,
    PAGE4= 0x400,
    PAGE5= 0x500,
    PAGE6= 0x600,
    PAGE7= 0x700
};

typedef enum SC_FEATURE
{
     SC_OUT_0_1 = 0x30 + PAGE0,  // num steps lo word
     SC_OUT_2_3 = 0x32 + PAGE0,  // num steps hi word
     ACT_OUT = 0x34 + PAGE0,     // still, walking, running
     SC_1= 0x30 + PAGE3,
     SC_2= 0x32 + PAGE3,
     SC_3= 0x34 + PAGE3,
     SC_4= 0x36 + PAGE3,
     SC_5= 0x38 + PAGE3,
     SC_6= 0x3A + PAGE3,
     SC_7= 0x3C + PAGE3,
     SC_8= 0x3E + PAGE3,
     SC_9= 0x30 + PAGE4,
    SC_10= 0x32 + PAGE4,
    SC_11= 0x34 + PAGE4,
    SC_12= 0x36 + PAGE4,
    SC_13= 0x38 + PAGE4,
    SC_14= 0x3A + PAGE4,
    SC_15= 0x3C + PAGE4,
    SC_16= 0x3E + PAGE4,
    SC_17= 0x30 + PAGE5,
    SC_18= 0x32 + PAGE5,
    SC_19= 0x34 + PAGE5,
    SC_20= 0x36 + PAGE5,
    SC_21= 0x38 + PAGE5,
    SC_22= 0x3A + PAGE5,
    SC_23= 0x3C + PAGE5,
    SC_24= 0x3E + PAGE5,
    SC_25= 0x30 + PAGE6,
    SC_26= 0x32 + PAGE6,
    SC_27= 0x34 + PAGE6
};

typedef enum WAKEUP_FEATURE
{
    WR_WAKEUP_1= 0x30 + PAGE7,
    WR_WAKEUP_2= 0x32 + PAGE7,
    WR_WAKEUP_3= 0x34 + PAGE7,
    WR_WAKEUP_4= 0x36 + PAGE7,
    WR_WAKEUP_5= 0x38 + PAGE7,
    WR_WAKEUP_6= 0x3A + PAGE7,
    WR_WAKEUP_7= 0x3C + PAGE7
};

typedef enum GESTURE_FEATURE
{
    WR_GEST_OUT = 0x36 + PAGE0,
    WR_GEST_1= 0x36 + PAGE6,
    WR_GEST_2= 0x38 + PAGE6,
    WR_GEST_3= 0x3A + PAGE6,
    WR_GEST_4= 0x3C + PAGE6,
};


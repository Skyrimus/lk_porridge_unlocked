#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <stdio.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/kernel.h>
#endif
// ---------------------------------------------------------------------------

#ifdef BUILD_LK
	#include <mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mt-plat/mt_gpio.h>
#endif

//  Local Constants
// ---------------------------------------------------------------------------
#define	FRAME_WIDTH		(720)
#define FRAME_HEIGHT		(1280)


#define   LCM_HSYNC_NUM		(20)  
#define   LCM_HBP_NUM		(80)  
#define   LCM_HFP_NUM		(80) 
    
#define   LCM_VSYNC_NUM		(2)  
#define   LCM_VBP_NUM		(13)  
#define   LCM_VFP_NUM		(16)   

#define   LCM_LINE_BYTE		((FRAME_WIDTH+LCM_HSYNC_NUM+LCM_HBP_NUM+LCM_HFP_NUM)*3)

#define REGFLAG_DELAY		0XFE
#define REGFLAG_END_OF_TABLE	0xFF   // END OF REGISTERS MARKER

#define _LCM_DEBUG_

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifdef _LCM_DEBUG_
  #ifdef BUILD_LK
  #define LCM_PRINT printf
  #else
  #define LCM_PRINT printk
  #endif
#else
	#define LCM_PRINT
#endif

#ifndef BUILD_LK
//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)  lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)

#define wrtie_cmd(cmd)			       lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)     lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)			       lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)  lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define set_gpio_lcd_enp(cmd) \
		lcm_util.set_gpio_lcd_enp_bias(cmd)
#define set_gpio_lcd_enn(cmd) \
		lcm_util.set_gpio_lcd_enn_bias(cmd)

#define GPIO_LCD_BIAS_ENP_PIN	(100|0x80000000)
#define GPIO_LCD_BIAS_ENN_PIN	(108|0x80000000)
#define GPIO_LCM_BL_EN	(63|0x80000000)

static bool is_lcm_suspend = 0;

/*****************************************************************************
** Gate driver function
******************************************************************************/
int lcm_gate_write_bytes(unsigned char addr, unsigned char value);

static void lcm_gate_enable(int enable)
{
    
    LCM_PRINT("[Kernel] %s = %d \n", __func__, enable);
    if( TRUE == enable )
    {
    	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
        MDELAY(5);
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
        MDELAY(5);
    }
    else
    {
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	//[SM31][RaymondLin]Disable LCM +-5V voltage when system suspend begin 
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
	//[SM31][RaymondLin]Disable LCM +-5V voltage when system suspend end
        MDELAY(5);
    	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	//[SM31][RaymondLin]Disable LCM +-5V voltage when system suspend begin 
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
	//[SM31][RaymondLin]Disable LCM +-5V voltage when system suspend end
        MDELAY(5);
    }
}

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};


static struct LCM_setting_table lcm_init_setting[] = 
{
// page 1
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x00}},
	{0xB1, 2, {0x60,0x27}},
//[SM31][RaymondLin]ESD solution from vendor begin
 {0x62, 1, {0x01}},
 {0x64, 1, {0x07}},
 {0xF0, 5, {0x55,0xAA,0x52,0x08,0x00}},
  {0xB1, 1, {0x6C}},
//[SM31][RaymondLin]ESD solution from vendor end	
  //Sleep Out
  {0x11, 0, {}},
  {REGFLAG_DELAY, 120, {}}, 

  //Display ON
  {0x29, 0, {}},
  {REGFLAG_DELAY, 20, {}}, 

  // Setting ending by predefined flag
  {REGFLAG_END_OF_TABLE, 0x00, {}}
};

//[SM31][RaymondLin] LCM into deep standby mode when system suspend - begin 
static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },// Display Off
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },// Dleep In
	{0xB0, 1, {0x00} },
	{0xB1, 1, {0x01} },//Deep standby In
	{REGFLAG_DELAY, 80, {} },
};
//[SM31][RaymondLin] LCM into deep standby mode when system suspend - end
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
		dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));
	
    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
    params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
	
    // DSI
    /* Command mode setting */
    //1 Three lane or Four lane
    params->dsi.LANE_NUM		= LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Video mode setting		
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active	= LCM_VSYNC_NUM;
    params->dsi.vertical_backporch	= LCM_VBP_NUM;
    params->dsi.vertical_frontporch	= LCM_VFP_NUM;
    params->dsi.vertical_active_line	= FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active	= LCM_HSYNC_NUM;
    params->dsi.horizontal_backporch	= LCM_HBP_NUM;
    params->dsi.horizontal_frontporch	= LCM_HFP_NUM;
    params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

    //params->dsi.LPX=8; 

    // Bit rate calculation
    params->dsi.ssc_range 	= 4;
    params->dsi.ssc_disable	= 1;
    //params->dsi.PLL_CLOCK	= 231;// 240;
//[SM31][RaymondLin]Fix LCM horizontal white line issue begin 	
    params->dsi.PLL_CLOCK = 220;//240;
//[SM31][RaymondLin]Fix LCM horizontal white line issue end	
//[SM31][RaymondLin]Enable ESD recovery begin
    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable      = 1;
    params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
//[SM31][RaymondLin]Enable ESD recovery end
}

static void lcm_init(void)
{
    lcm_gate_enable(1);

    SET_RESET_PIN(1);
    MDELAY(2); 
    SET_RESET_PIN(0);
    MDELAY(5); 
    SET_RESET_PIN(1);
    MDELAY(5); 

    push_table(lcm_init_setting, sizeof(lcm_init_setting) / sizeof(struct LCM_setting_table), 1);
    //dsi_set_cmdq_V3(lcm_init_setting,sizeof(lcm_init_setting)/sizeof(lcm_init_setting[0]),1);
    LCM_PRINT("lcm init in kernel is done \n");
}



static void lcm_suspend(void)
{
//[SM31][RaymondLin] LCM into deep standby mode when system suspend - begin
    //unsigned int data_array[16];
//[SM31][RaymondLin] LCM into deep standby mode when system suspend - end	
LCM_PRINT("[Kernel] %s ++\n", __func__);
//[SM31][RaymondLin]Disable BL_EN pin when system suspend -begin
	mt_set_gpio_mode(GPIO_LCM_BL_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_BL_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_BL_EN, GPIO_OUT_ZERO);
//[SM31][RaymondLin]Disable BL_EN pin when system suspend -begin
//[SM31][RaymondLin] LCM into deep standby mode when system suspend - begin
push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
MDELAY(10);
    
//[SM31][RaymondLin] LCM into deep standby mode when system suspend - end
    is_lcm_suspend = 1;
    lcm_gate_enable(0);
LCM_PRINT("[Kernel] %s --\n", __func__);	
}

static void lcm_resume(void)
{
    LCM_PRINT("[Kernel] %s ++\n",__func__);
	lcm_init();

    is_lcm_suspend = 0;

//[SM31][RaymondLin]Enable BL_EN pin when system resume -begin
	mt_set_gpio_mode(GPIO_LCM_BL_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_BL_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_BL_EN, GPIO_OUT_ONE);
//[SM31][RaymondLin]Enable BL_EN pin when system suspend -begin	
    LCM_PRINT("[Kernel] %s --\n",__func__);
}

static unsigned int lcm_compare_id(void)
{
     return 1;
}

LCM_DRIVER nt35521s_hd720_dsi_vdo_lcm_drv = 
{
    .name			= "nt35521s_hd720_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
    };

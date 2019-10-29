LOCAL_DIR := $(GET_LOCAL_DIR)
TARGET := fih6753_65_n
MODULES += app/mt_boot \
           dev/lcm
MTK_EMMC_SUPPORT = yes
DEFINES += MTK_NEW_COMBO_EMMC_SUPPORT
MTK_KERNEL_POWER_OFF_CHARGING = yes
MTK_PUMP_EXPRESS_SUPPORT := yes
MTK_LCM_PHYSICAL_ROTATION = 0
CUSTOM_LK_LCM="r63417_fhd_dsi_cmd_truly_nt50358"
#nt35590_hd720_dsi_cmd_auo = yes
DEFINES += MTK_LCM_DEVICE_TREE_SUPPORT
#FASTBOOT_USE_G_ORIGINAL_PROTOCOL = yes
MTK_SECURITY_SW_SUPPORT = yes
MTK_VERIFIED_BOOT_SUPPORT = yes
MTK_SEC_FASTBOOT_UNLOCK_SUPPORT = yes
DEBUG := 0
BOOT_LOGO:=cu_lte_fhd
#DEFINES += WITH_DEBUG_DCC=1
DEFINES += WITH_DEBUG_UART=1
#DEFINES += WITH_DEBUG_FBCON=1
#DEFINES += MACH_FPGA=y
#DEFINES += SB_LK_BRINGUP=y
DEFINES += MTK_GPT_SCHEME_SUPPORT
MTK_GOOGLE_TRUSTY_SUPPORT=no

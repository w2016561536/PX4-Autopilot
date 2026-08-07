ENTRY(__start);
_diram_i_start = 0x40378000;
SECTIONS
{
  .iram0.vectors : ALIGN(4)
  {
    _iram_start = ABSOLUTE(.);
    _init_start = ABSOLUTE(.);
    . = 0x0;
    KEEP (*(.window_vectors.text));
    . = 0x180;
    KEEP (*(.xtensa_level2_vector.text));
    . = 0x1c0;
    KEEP (*(.xtensa_level3_vector.text));
    . = 0x200;
    KEEP (*(.xtensa_level4_vector.text));
    . = 0x240;
    KEEP (*(.xtensa_level5_vector.text));
    . = 0x280;
    KEEP (*(.debug_exception_vector.text));
    . = 0x2c0;
    KEEP (*(.nmi_vector.text));
    . = 0x300;
    KEEP (*(.kernel_exception_vector.text));
    . = 0x340;
    KEEP (*(.user_exception_vector.text));
    . = 0x3c0;
    KEEP (*(.double_exception_vector.text));
    . = 0x400;
    *(.*_vector.literal)
    . = ALIGN(16);
    *(.entry.text)
    *(.init.literal)
    *(.init)
    _init_end = ABSOLUTE(.);
  } >iram0_0_seg AT>ROM
  .iram0.text : ALIGN(4)
  {
    *(.iram1 .iram1.*)
    esp32s3_start.*(.literal .text .literal.* .text.*)
    esp32s3_region.*(.literal .text .literal.* .text.*)
    *libcoexist.a:(.coexiram .coexiram.*)
    *libpp.a:wifi_iram.*(.literal .text .literal.* .text.*)
    *libpp.a:wifi_rx_iram.*(.literal .text .literal.* .text.*)
    *libpp.a:wifi_extra_iram.*(.literal .text .literal.* .text.*)
    *libpp.a:wifi_slp_iram.*(.literal .text .literal.* .text.*)
    *libpp.a:wifi_or_slp_iram.*(.literal .text .literal.* .text.*)
    *libpp.a:wifi_slp_rx_iram.*(.literal .text .literal.* .text.*)
    *libarch.a:*esp_loader.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32s3_cpuindex.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32s3_irq.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32s3_user.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32s3_spiflash.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_assert.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_cpuint.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_cpupause.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_irqdispatch.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_modifyreg32.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_testset.*(.literal .text .literal.* .text.*)
    *libarch.a:esp_app_desc.*(.literal .text .literal.* .text.*)
    *libc.a:sq_remlast.*(.literal .text .literal.* .text.*)
    *libdrivers.a:syslog_flush.*(.literal .text .literal.* .text.*)
    *libsched.a:assert.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_csection.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_dispatch.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_spinlock.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_lock.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_note.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_suspendscheduler.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_thistask.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_unlock.*(.literal .text .literal.* .text.*)
    *libsched.a:spinlock.*(.literal .text .literal.* .text.*)
    *libsched.a:*sched_get_stackinfo.*(.literal .text .literal.* .text.*)
    *libsched.a:stack_record.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32s3_spi_timing.*(.literal .text .literal.* .text.*)
    *libarch.a:*brownout_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*cpu.*(.text .text.* .literal .literal.*)
    *libarch.a:*gpio_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*periph_ctrl.*(.text .text.* .literal .literal.*)
    *libarch.a:*clk.*(.text .text.* .literal .literal.*)
    *libarch.a:*efuse_hal.*(.literal.is_eco0 .text.is_eco0)
    *libarch.a:*esp_clk.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_clk_tree.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_clk_tree_common.*(.text .text.* .literal .literal.*)
    *libarch.a:*clk_tree_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*rtc_init.*(.text .text.* .literal .literal.*)
    *libarch.a:*rtc_clk.*(.text .text.* .literal .literal.*)
    *libarch.a:*rtc_clk_init.*(.text .text.* .literal .literal.*)
    *libarch.a:*rtc_sleep.*(.text .text.* .literal .literal.*)
    *libarch.a:*rtc_time.*(.text .text.* .literal .literal.*)
    *libarch.a:*regi2c_ctrl.*(.text .text.* .literal .literal.*)
    *libarch.a:*uart_hal_iram.*(.text .text.* .literal .literal.*)
    *libarch.a:*wdt_hal_iram.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_banner_wrap.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_init.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_common.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_common_loader.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_console.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_console_loader.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_esp32s3.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_flash.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_flash_config_esp32s3.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_clock_init.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_clock_loader.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_efuse.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_panic.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_mem.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_random.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_random*.*(.literal.bootloader_random_disable .text.bootloader_random_disable)
    *libarch.a:*bootloader_random*.*(.literal.bootloader_random_enable .text.bootloader_random_enable)
    *libarch.a:*bootloader_random_esp32s3.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_image_format.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_soc.*(.text .text.* .literal .literal.*)
    *libarch.a:*bootloader_sha.*(.text .text.* .literal .literal.*)
    *libarch.a:*flash_encrypt.*(.text .text.* .literal .literal.*)
    *libarch.a:*cache_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*uart_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*mpu_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*mmu_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*efuse_hal.*(.text .text.* .literal .literal.*)
    *libarch.a:*uart_periph.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_uart.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_sys.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_spiflash.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_cache_esp32s2_esp32s3.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_wdt.*(.text .text.* .literal .literal.*)
    *libarch.a:*log.*(.text .text.* .literal .literal.*)
    *libarch.a:*log_lock.*(.literal .literal.* .text .text.*)
    *libarch.a:*log_print.*(.literal .literal.* .text .text.*)
    *libarch.a:*log_timestamp.*(.literal.esp_log_early_timestamp .text.esp_log_early_timestamp)
    *libarch.a:*log_timestamp.*(.literal.esp_log_timestamp .text.esp_log_timestamp)
    *libarch.a:*log_timestamp_common.*(.literal .literal.* .text .text.*)
    *libarch.a:*log_write.*(.literal.esp_log_write .text.esp_log_write)
    *libarch.a:*log_write.*(.literal.esp_log_writev .text.esp_log_writev)
    *libarch.a:*cpu_region_protect.*(.text .text.* .literal .literal.*)
    *libarch.a:*mspi_timing_tuning.*(.text .text.* .literal .literal.*)
    *libarch.a:*mspi_timing_config.*(.text .text.* .literal .literal.*)
    *libarch.a:*esp_rom_cache_esp32s2_esp32s3.*(.literal .text .literal.* .text.*)
    *libarch.a:*flash_qio_mode.*(.text .text.* .literal .literal.*)
    *libarch.a:*spi_flash_wrap.*(.text .text.* .literal .literal.*)
    *libarch.a:*spi_flash_oct_flash_init.*(.text .text.* .literal .literal.*)
    *libarch.a:*spi_flash_hpm_enable.*(.text .text.* .literal .literal.*)
    *libarch.a:esp_spiflash.*(.literal .text .literal.* .text.*)
    *libarch.a:esp_flash_api.*(.text .text.* .literal .literal.*)
    *libarch.a:esp_flash_spi_init.*(.text .text.* .literal .literal.*)
    *libarch.a:spi_flash_hal_iram.*(.literal .literal.* .text .text.*)
    *libarch.a:spi_flash_encrypt_hal_iram.*(.text .text.* .literal .literal.*)
    *libarch.a:spi_flash_hal_gpspi.*(.literal .literal.* .text .text.*)
    *libarch.a:spi_flash_chip*.*(.literal .literal.* .text .text.*)
    *libarch.a:spi_flash_wrap.*(.literal .literal.* .text .text.*)
    *libarch.a:spi_flash_os_func_noos.*(.literal .literal.* .text .text.*)
    *libarch.a:spi_flash_os_func_app.*(.literal .literal.* .text .text.*)
    *libarch.a:flash_brownout_hook.*(.literal .literal.* .text .text.*)
    *libarch.a:esp_cache.*(.literal .literal.* .text .text.*)
    *libarch.a:cache_utils.*(.literal .literal.* .text .text.*)
    *libarch.a:memspi_host_driver.*(.literal .literal.* .text .text.*)
    *libc.a:*lib_instrument.*(.text .text.* .literal .literal.*)
    *(.wifirxiram .wifirxiram.*)
    *(.wifi0iram .wifi0iram.*)
    *(.wifiorslpiram .wifiorslpiram.*)
    *(.wifislpiram .wifislpiram.*)
    *(.wifislprxiram .wifislprxiram.*)
    *(.phyiram .phyiram.*)
    . = ALIGN(4) + 16;
    _iram_text = ABSOLUTE(.);
  } >iram0_0_seg
  .iram0.text_end (NOLOAD) :
  {
    . += 16;
    . = ALIGN(256);
    _iram_end = ABSOLUTE(.);
  } >iram0_0_seg
  .iram0.data :
  {
    . = ALIGN(4);
    *(.iram.data)
    *(.iram.data.*)
  } >iram0_0_seg
  .iram0.bss (NOLOAD) :
  {
    . = ALIGN(4);
    *(.iram.bss)
    *(.iram.bss.*)
    . = ALIGN(4);
    _iram_end = ABSOLUTE(.);
  } >iram0_0_seg
  .dram0.dummy (NOLOAD) :
  {
    . = ORIGIN(dram0_0_seg) + MAX(_iram_end, _diram_i_start) - _diram_i_start;
  } >dram0_0_seg
  .dram0.bss (NOLOAD) :
  {
    . = ALIGN(8);
    _bss_start = ABSOLUTE(.);
    _sbss = ABSOLUTE(.);
    *(.bss .bss.*)
    *(COMMON)
    *(.dynsbss)
    *(.sbss)
    *(.sbss.*)
    *(.gnu.linkonce.sb.*)
    *(.scommon)
    *(.sbss2)
    *(.sbss2.*)
    *(.gnu.linkonce.sb2.*)
    *(.dynbss)
    *(.share.mem)
    *(.gnu.linkonce.b.*)
    . = ALIGN(8);
    _ebss = ABSOLUTE(.);
    _bss_end = ABSOLUTE(.);
  } >dram0_0_seg
  .noinit (NOLOAD) :
  {
    . = ALIGN(4);
    *(.noinit .noinit.*)
    . = ALIGN(4);
  } >dram0_0_seg
  .dram0.data : ALIGN(4)
  {
    . = ALIGN (16);
    _data_start = ABSOLUTE(.);
    _sdata = ABSOLUTE(.);
    KEEP (*(.data))
    KEEP (*(.data.*))
    KEEP (*(.gnu.linkonce.d.*))
    KEEP (*(.data1))
    KEEP (*(.sdata))
    KEEP (*(.sdata.*))
    KEEP (*(.gnu.linkonce.s.*))
    KEEP (*(.sdata2))
    KEEP (*(.sdata2.*))
    KEEP (*(.gnu.linkonce.s2.*))
    KEEP (*(.jcr))
    *(.dram1 .dram1.*)
    esp32s3_start.*(.rodata .rodata.*)
    esp32s3_region.*(.rodata .rodata.*)
    *libphy.a:(.rodata .rodata.*)
    *libarch.a:xtensa_context.*(.rodata .rodata.*)
    *libsched.a:*sched_get_stackinfo.*(.rodata .rodata.*)
    *libarch.a:*esp_loader.*(.rodata .rodata.*)
    *libarch.a:esp32s3_spiflash.*(.rodata .rodata.*)
    *libarch.a:esp_spiflash.*(.rodata .rodata.*)
    *libarch.a:*brownout.*(.rodata .rodata.*)
    *libarch.a:*cpu.*(.rodata .rodata.*)
    *libarch.a:*gpio_hal.*(.rodata .rodata.*)
    *libarch.a:*periph_ctrl.*(.rodata .rodata.*)
    *libarch.a:*clk.*(.rodata .rodata.*)
    *libarch.a:*esp_clk.*(.rodata .rodata.*)
    *libarch.a:*esp_clk_tree.*(.rodata .rodata.*)
    *libarch.a:*esp_clk_tree_common.*(.rodata .rodata.*)
    *libarch.a:*clk_tree_hal.*(.rodata .rodata.*)
    *libarch.a:*rtc_init.*(.rodata .rodata.*)
    *libarch.a:*rtc_clk.*(.rodata .rodata.*)
    *libarch.a:*rtc_clk_init.*(.rodata .rodata.*)
    *libarch.a:*rtc_sleep.*(.rodata .rodata.*)
    *libarch.a:*rtc_time.*(.rodata .rodata.*)
    *libarch.a:*regi2c_ctrl.*(.rodata .rodata.*)
    *libarch.a:*uart_hal_iram.*(.rodata .rodata.*)
    *libarch.a:*wdt_hal_iram.*(.rodata .rodata.*)
    *libarch.a:*bootloader_banner_wrap.*(.rodata .rodata.*)
    *libarch.a:*bootloader_init.*(.rodata .rodata.*)
    *libarch.a:*bootloader_common.*(.rodata .rodata.*)
    *libarch.a:*bootloader_common_loader.*(.rodata .rodata.*)
    *libarch.a:*bootloader_console.*(.rodata .rodata.*)
    *libarch.a:*bootloader_console_loader.*(.rodata .rodata.*)
    *libarch.a:*bootloader_esp32s3.*(.rodata .rodata.*)
    *libarch.a:*bootloader_flash.*(.rodata .rodata.*)
    *libarch.a:*bootloader_flash_config_esp32s3.*(.rodata .rodata.*)
    *libarch.a:*bootloader_clock_init.*(.rodata .rodata.*)
    *libarch.a:*bootloader_clock_loader.*(.rodata .rodata.*)
    *libarch.a:*bootloader_efuse.*(.rodata .rodata.*)
    *libarch.a:*bootloader_panic.*(.rodata .rodata.*)
    *libarch.a:*bootloader_mem.*(.rodata .rodata.*)
    *libarch.a:*bootloader_random.*(.rodata .rodata.*)
    *libarch.a:*bootloader_random_esp32s3.*(.rodata .rodata.*)
    *libarch.a:*esp_image_format.*(.rodata .rodata.*)
    *libarch.a:*bootloader_soc.*(.rodata .rodata.*)
    *libarch.a:*bootloader_sha.*(.rodata .rodata.*)
    *libarch.a:*flash_encrypt.*(.rodata .rodata.*)
    *libarch.a:*cache_hal.*(.rodata .rodata.*)
    *libarch.a:*uart_hal.*(.rodata .rodata.*)
    *libarch.a:*mpu_hal.*(.rodata .rodata.*)
    *libarch.a:*mmu_hal.*(.rodata .rodata.*)
    *libarch.a:*uart_periph.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_uart.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_sys.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_spiflash.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_cache_esp32s2_esp32s3.*(.rodata .rodata.*)
    *libarch.a:*esp_rom_wdt.*(.rodata .rodata.*)
    *libarch.a:*efuse_hal.*(.rodata .rodata.*)
    *libarch.a:*log.*(.rodata .rodata.*)
    *libarch.a:*log_noos.*(.rodata .rodata.*)
    *libarch.a:*cpu_region_protect.*(.rodata .rodata.*)
    *libarch.a:*mspi_timing_tuning.*(.rodata .rodata.*)
    *libarch.a:*mspi_timing_config.*(.rodata .rodata.*)
    *libarch.a:*flash_qio_mode.*(.rodata .rodata.*)
    *libarch.a:*spi_flash_wrap.*(.rodata .rodata.*)
    *libarch.a:*spi_flash_oct_flash_init.*(.rodata .rodata.*)
    *libarch.a:*spi_flash_hpm_enable.*(.rodata .rodata.*)
    *libarch.a:esp_flash_api.*(.rodata .rodata.*)
    *libarch.a:esp_flash_spi_init.*(.rodata .rodata.*)
    *libarch.a:spi_flash_hal_iram.*(.rodata .rodata.*)
    *libarch.a:spi_flash_encrypt_hal_iram.*(.rodata .rodata.*)
    *libarch.a:spi_flash_hal_gpspi.*(.rodata .rodata.*)
    *libarch.a:spi_flash_chip*.*(.rodata .rodata.*)
    *libarch.a:spi_flash_wrap.*(.rodata .rodata.*)
    *libarch.a:spi_flash_os_func_noos.*(.rodata .rodata.*)
    *libarch.a:spi_flash_os_func_app.*(.rodata .rodata.*)
    *libarch.a:flash_brownout_hook.*(.rodata .rodata.*)
    *libarch.a:esp_cache.*(.rodata .rodata.*)
    *libarch.a:cache_utils.*(.rodata .rodata.*)
    *libarch.a:memspi_host_driver.*(.rodata .rodata.*)
    . = ALIGN(4);
    _edata = ABSOLUTE(.);
    _data_end = ABSOLUTE(.);
    _sheap = ABSOLUTE(.);
  } >dram0_0_seg AT>ROM
  .xt.prop 0 :
  {
    KEEP (*(.xt.prop .gnu.linkonce.prop.*))
  }
  .xt.lit 0 :
  {
    KEEP (*(.xt.lit .gnu.linkonce.p.*))
  }
  .flash.text : ALIGN(0x00010000)
  {
    _stext = .;
    _instruction_reserved_start = ABSOLUTE(.);
    *(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.irom0.text)
    *(.fini.literal)
    *(.fini)
    *(.gnu.version)
    *(EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifi0iram EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifi0iram.*)
    *(.wifiextrairam .wifiextrairam.*)
    *(EXCLUDE_FILE(*libpp.a) .wifiorslpiram EXCLUDE_FILE(*libpp.a) .wifiorslpiram.*)
    *(EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifirxiram EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifirxiram.*)
    *(.wifislpiram .wifislpiram.*)
    *(EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifislprxiram EXCLUDE_FILE(*libnet80211.a *libpp.a) .wifislprxiram.*)
    . += 16;
    _instruction_reserved_end = ABSOLUTE(.);
    _etext = .;
  } >irom0_0_seg AT>ROM
  _image_irom_vma = ADDR(.flash.text);
  _image_irom_lma = LOADADDR(.flash.text);
  _image_irom_size = LOADADDR(.flash.text) + SIZEOF(.flash.text) - _image_irom_lma;
  .flash.rodata_dummy (NOLOAD) :
  {
    _flash_rodata_dummy_start = ABSOLUTE(.);
    . = ALIGN(ALIGNOF(.flash.text)) + SIZEOF(.flash.text);
    . = ALIGN(0x10000) + 0x20;
  } > drom0_0_seg
  .flash.rodata : ALIGN(0x10000)
  {
    _rodata_reserved_start = ABSOLUTE(.);
    *(.rodata_desc .rodata_desc.*)
    _srodata = ABSOLUTE(.);
    *(EXCLUDE_FILE (esp32s3_start.* esp32s3_region.*
                    *libarch.a:*esp_loader.*
                    *libarch.a:esp32s3_spiflash.*
                    *libarch.a:*cache_hal.* *libarch.a:*mmu_hal.*
                    *libarch.a:*mpu_hal.*) .rodata)
    *(EXCLUDE_FILE (esp32s3_start.* esp32s3_region.*
                    *libarch.a:*esp_loader.*
                    *libarch.a:esp32s3_spiflash.*
                    *libarch.a:*cache_hal.* *libarch.a:*mmu_hal.*
                    *libarch.a:*mpu_hal.*) .rodata.*)
    *(.rodata_wlog_verbose.*)
    *(.rodata_wlog_debug.*)
    *(.rodata_wlog_info.*)
    *(.rodata_wlog_warning.*)
    *(.rodata_wlog_error.*)
    *(.irom1.text)
    *(.gnu.linkonce.r.*)
    *(.rodata1)
    __XT_EXCEPTION_TABLE_ = ABSOLUTE(.);
    *(.xt_except_table)
    *(.gcc_except_table)
    *(.gcc_except_table.*)
    *(.gnu.linkonce.e.*)
    *(.gnu.version_r)
    *(.eh_frame)
    . = ALIGN(4);
    __XT_EXCEPTION_DESCS_ = ABSOLUTE(.);
    *(.xt_except_desc)
    *(.gnu.linkonce.h.*)
    __XT_EXCEPTION_DESCS_END__ = ABSOLUTE(.);
    *(.xt_except_desc_end)
    *(.dynamic)
    *(.gnu.version_d)
    _erodata = ABSOLUTE(.);
    _lit4_start = ABSOLUTE(.);
    *(*.lit4)
    *(.lit4.*)
    *(.gnu.linkonce.lit4.*)
    _lit4_end = ABSOLUTE(.);
    . = ALIGN(4);
    _sinit = ABSOLUTE(.);
    __init_array_start = ABSOLUTE(.);
    KEEP (*(EXCLUDE_FILE (*crtend.* *crtbegin.*) .ctors SORT(.ctors.*)))
    __init_array_end = ABSOLUTE(.);
    _einit = ABSOLUTE(.);
    . = ALIGN(4);
    soc_reserved_memory_region_start = ABSOLUTE(.);
    KEEP (*(.reserved_memory_address))
    soc_reserved_memory_region_end = ABSOLUTE(.);
    . = ALIGN(4);
    _esp_system_init_fn_array_start = ABSOLUTE(.);
    KEEP (*(SORT_BY_INIT_PRIORITY(.esp_system_init_fn.*)))
    _esp_system_init_fn_array_end = ABSOLUTE(.);
    . = ALIGN(4);
    _thread_local_start = ABSOLUTE(.);
    _stdata = ABSOLUTE(.);
    *(.tdata .tdata.* .gnu.linkonce.td.*);
    _etdata = ABSOLUTE(.);
    _stbss = ABSOLUTE(.);
    *(.tbss .tbss.* .gnu.linkonce.tb.* .tcommon);
    _etbss = ABSOLUTE(.);
    _thread_local_end = ABSOLUTE(.);
    _rodata_reserved_end = ABSOLUTE(.);
  } >drom0_0_seg AT>ROM
  _rodata_reserved_align = ALIGNOF(.flash.rodata);
  _image_drom_vma = ADDR(.flash.rodata);
  _image_drom_lma = LOADADDR(.flash.rodata);
  _image_drom_size = LOADADDR(.flash.rodata) + SIZEOF(.flash.rodata) - _image_drom_lma;
  .rtc.text :
  {
    . = ALIGN(4);
    _rtc_fast_start = ABSOLUTE(.);
    _rtc_text_start = ABSOLUTE(.);
    *(.rtc.literal .rtc.text)
    _rtc_text_end = ABSOLUTE(.);
  } >rtc_iram_seg AT>ROM
  .rtc.dummy (NOLOAD) :
  {
    . = SIZEOF(.rtc.text);
  } >rtc_slow_seg
  .rtc.bss (NOLOAD) :
  {
    *(.rtc.bss)
  } >rtc_slow_seg
  .rtc.force_fast :
  {
    . = ALIGN(4);
    _rtc_force_fast_start = ABSOLUTE(.);
    *(.rtc.force_fast .rtc.force_fast.*)
    . = ALIGN(4);
    _rtc_force_fast_end = ABSOLUTE(.);
  } >rtc_slow_seg AT>ROM
  .rtc.data :
  {
    *(.rtc.data)
    *(.rtc.data.*)
    *(.rtc.rodata)
    *(.rtc.rodata.*)
    . = ALIGN (4);
  } >rtc_slow_seg AT>ROM
  .rtc.heap : ALIGN(4)
  {
    _srtcheap = ABSOLUTE(.);
  } >rtc_slow_seg
  .rtc_reserved (NOLOAD):
  {
    . = ALIGN(4);
    _rtc_reserved_start = ABSOLUTE(.);
    *(.rtc_timer_data_in_rtc_mem .rtc_timer_data_in_rtc_mem.*)
    KEEP(*(.bootloader_data_rtc_mem .bootloader_data_rtc_mem.*))
    _rtc_reserved_end = ABSOLUTE(.);
  } > rtc_reserved_seg
}

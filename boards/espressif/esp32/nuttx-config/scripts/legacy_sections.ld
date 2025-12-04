ENTRY(__start);
SECTIONS
{
  .iram0.vectors :
  {
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
    . = ALIGN (16);
    *(.entry.text)
    *(.init.literal)
    *(.init)
    _init_end = ABSOLUTE(.);
  } > iram0_0_seg
  .iram0.text :
  {
    _iram_text_start = ABSOLUTE(.);
    *(.iram1 .iram1.*)
    *librtc.a:(.literal .text .literal.* .text.*)
    *libarch.a:esp32_cpuindex.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32_irq.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32_spicache.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32_spiflash.*(.literal .text .literal.* .text.*)
    *libarch.a:esp32_user.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_assert.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_cpuint.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_cpupause.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_irqdispatch.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_modifyreg32.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_saveusercontext.*(.literal .text .literal.* .text.*)
    *libarch.a:xtensa_testset.*(.literal .text .literal.* .text.*)
    *libc.a:lib_assert.*(.literal .text .literal.* .text.*)
    *libc.a:lib_utsname.*(.literal .text .literal.* .text.*)
    *libc.a:sq_remlast.*(.literal .text .literal.* .text.*)
    *libdrivers.a:syslog_flush.*(.literal .text .literal.* .text.*)
    *libsched.a:assert.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_csection.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_dispatch.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_spinlock.*(.literal .text .literal.* .text.*)
    *libsched.a:panic_notifier.*(.literal.panic_notifier_call_chain .text.panic_notifier_call_chain)
    *libsched.a:sched_gettcb.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_lock.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_note.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_suspendscheduler.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_thistask.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_unlock.*(.literal .text .literal.* .text.*)
    *libsched.a:spinlock.*(.literal .text .literal.* .text.*)
    *(.wifirxiram .wifirxiram.*)
    *(.wifi0iram .wifi0iram.*)
    *(.wifiorslpiram .wifiorslpiram.*)
    *(.wifislpiram .wifislpiram.*)
    *(.wifislprxiram .wifislprxiram.*)
    *(.phyiram .phyiram.*)
    _iram_text_end = ABSOLUTE(.);
    . = ALIGN (4);
    _siramheap = ABSOLUTE(.);
  } > iram0_0_seg
  .dram0.bss (NOLOAD) :
  {
    . = ALIGN (8);
    _sbss = ABSOLUTE(.);
    _bss_start = ABSOLUTE(.);
    *(EXCLUDE_FILE(*libble_app.a *libbt.a *libbtdm_app.a *libnimble.a) .bss EXCLUDE_FILE(*libble_app.a *libbt.a *libbtdm_app.a *libnimble.a) .bss.*)
    *(.ext_ram.bss*)
    *(EXCLUDE_FILE(*libble_app.a *libbt.a *libbtdm_app.a *libnimble.a) COMMON)
    . = ALIGN(4);
    _bt_controller_bss_start = ABSOLUTE(.);
    *libble_app.a:(.bss .bss.*)
    . = ALIGN(4);
    _bt_controller_bss_end = ABSOLUTE(.);
    . = ALIGN(4);
    _bt_controller_common_start = ABSOLUTE(.);
    *libble_app.a:(COMMON)
    . = ALIGN(4);
    _bt_controller_common_end = ABSOLUTE(.);
    . = ALIGN(4);
    _bt_bss_start = ABSOLUTE(.);
    *libbt.a:(.bss .bss.*)
    . = ALIGN(4);
    _bt_bss_end = ABSOLUTE(.);
    . = ALIGN(4);
    _bt_common_start = ABSOLUTE(.);
    *libbt.a:(COMMON)
    . = ALIGN(4);
    _bt_common_end = ABSOLUTE(.);
    . = ALIGN(4);
    _btdm_bss_start = ABSOLUTE(.);
    *libbtdm_app.a:(.bss .bss.*)
    . = ALIGN(4);
    _btdm_bss_end = ABSOLUTE(.);
    . = ALIGN(4);
    _btdm_common_start = ABSOLUTE(.);
    *libbtdm_app.a:(COMMON)
    . = ALIGN(4);
    _btdm_common_end = ABSOLUTE(.);
    . = ALIGN (8);
    *(.dynsbss)
    *(.sbss)
    *(.sbss.*)
    *(.gnu.linkonce.sb.*)
    *(.scommon)
    *(.sbss2)
    *(.sbss2.*)
    *(.gnu.linkonce.sb2.*)
    *(.dynbss)
    KEEP (*(.bss))
    *(.bss.*)
    *(.share.mem)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    *libarch.a:esp32_spiflash.*(.bss .bss.* COMMON)
    *libarch.a:xtensa_cpupause.*(.bss .bss.* COMMON)
    *libarch.a:xtensa_copystate.*(.bss .bss.* COMMON)
    *libarch.a:xtensa_interruptcontext.*(.bss .bss.* COMMON)
    *libarch.a:xtensa_testset.*(.bss .bss.* COMMON)
    *libsched.a:sched_suspendscheduler.*(.bss .bss.* COMMON)
    *libsched.a:sched_thistask.*(.bss .bss.* COMMON)
    *libsched.a:sched_note.*(.bss .bss.* COMMON)
    *libsched.a:spinlock.*(.bss .bss.* COMMON)
    *libsched.a:irq_csection.*(.bss .bss.* COMMON)
    *libsched.a:irq_dispatch.*(.bss .bss.* COMMON)
    . = ALIGN(8);
    _bss_end = ABSOLUTE(.);
    _ebss = ABSOLUTE(.);
  } >dram0_0_seg
  .noinit (NOLOAD):
  {
    *(.noinit)
  } >dram0_0_seg
  .dram0.data :
  {
    _sdata = ABSOLUTE(.);
    _bt_data_start = ABSOLUTE(.);
    *libbt.a:(.data .data.*)
    . = ALIGN (4);
    _bt_data_end = ABSOLUTE(.);
    _btdm_data_start = ABSOLUTE(.);
    *libbtdm_app.a:(.data .data.*)
    . = ALIGN (4);
    _btdm_data_end = ABSOLUTE(.);
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
    *libphy.a:(.rodata .rodata.*)
    *libarch.a:esp32_spiflash.*(.rodata .rodata.*)
    *libarch.a:xtensa_context.*(.rodata .rodata.*)
    *libarch.a:xtensa_copystate.*(.rodata .rodata.*)
    *libarch.a:xtensa_cpupause.*(.rodata .rodata.*)
    *libarch.a:xtensa_testset.*(.rodata .rodata.*)
    *libdrivers.a:syslog_channel.*(.rodata .rodata.*)
    *libsched.a:sched_suspendscheduler.*(.rodata .rodata.*)
    *libsched.a:sched_thistask.*(.rodata .rodata.*)
    *libsched.a:sched_note.*(.rodata .rodata.*)
    *libsched.a:spinlock.*(.rodata .rodata.*)
    *libsched.a:irq_csection.*(.rodata .rodata.*)
    *libsched.a:irq_dispatch.*(.rodata .rodata.*)
    . = ALIGN(4);
    _edata = ABSOLUTE(.);
    _sheap = ABSOLUTE(.);
  } >dram0_0_seg
  .extmem.bss (NOLOAD) :
  {
    _sbss_extmem = ABSOLUTE(.);
    *(.extmem.bss .extmem.bss.*)
    . = ALIGN(4);
    _ebss_extmem = ABSOLUTE(.);
  } > extmem_seg
  .flash.rodata :
  {
    _srodata = ABSOLUTE(.);
    *(.rodata)
    *(.rodata.*)
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
    . = (. + 3) & ~ 3;
    _sinit = ABSOLUTE(.);
    KEEP (*crtbegin.o(.ctors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    _einit = ABSOLUTE(.);
    KEEP (*crtbegin.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    __XT_EXCEPTION_DESCS_ = ABSOLUTE(.);
    *(.xt_except_desc)
    *(.gnu.linkonce.h.*)
    __XT_EXCEPTION_DESCS_END__ = ABSOLUTE(.);
    *(.xt_except_desc_end)
    *(.dynamic)
    *(.gnu.version_d)
    soc_reserved_memory_region_start = ABSOLUTE(.);
    KEEP (*(.reserved_memory_address))
    soc_reserved_memory_region_end = ABSOLUTE(.);
    _erodata = ABSOLUTE(.);
    _lit4_start = ABSOLUTE(.);
    *(*.lit4)
    *(.lit4.*)
    *(.gnu.linkonce.lit4.*)
    _lit4_end = ABSOLUTE(.);
    . = ALIGN(4);
  } >default_rodata_seg
  .flash.text :
  {
    _stext = .;
    _text_start = ABSOLUTE(.);
    *(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.irom0.text)
    *(.fini.literal)
    *(.fini)
    *(.gnu.version)
    _text_end = ABSOLUTE(.);
    _etext = .;
  } >default_code_seg
  .rtc.text :
  {
    . = ALIGN(4);
    *(.rtc.literal .rtc.text)
  } >rtc_iram_seg
  .rtc.data :
  {
    *(.rtc.data)
    *(.rtc.data.*)
    *(.rtc.rodata)
    *(.rtc.rodata.*)
    . = ALIGN (4);
    _srtcheap = ABSOLUTE(.);
  } > rtc_slow_seg
}

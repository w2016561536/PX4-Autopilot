ENTRY(__start);
_diram_i_start = 0x40378000;
SECTIONS
{
  .iram0.vectors :
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
  } >iram0_0_seg
  .iram0.text :
  {
    *(.iram1 .iram1.*)
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
    *libdrivers.a:syslog_flush.*(.literal .text .literal.* .text.*)
    *libsched.a:assert.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_csection.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_dispatch.*(.literal .text .literal.* .text.*)
    *libsched.a:irq_spinlock.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_note.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_suspendscheduler.*(.literal .text .literal.* .text.*)
    *libsched.a:sched_thistask.*(.literal .text .literal.* .text.*)
    *libsched.a:spinlock.*(.literal .text .literal.* .text.*)

    *libarch.a:xtensa_switchcontext.*(.literal.up_switch_context .text.up_switch_context)

    *libarch.a:esp32s3_timerisr.*(.literal.systimer_isr .text.systimer_isr)
    *libarch.a:esp32s3_idle.*(.literal.up_idle .text.up_idle)
    *libarch.a:esp32s3_dma.*(.literal.esp32s3_dma_load .text.esp32s3_dma_load \
                             .literal.esp32s3_dma_enable .text.esp32s3_dma_enable)

    *libsched.a:sched_processtimer.*(.literal.nxsched_process_timer .text.nxsched_process_timer)
    *libsched.a:clock_initialize.*(.literal.clock_timer .text.clock_timer)
    *libsched.a:wd_start.*(.literal.wd_timer .text.wd_timer)
    *libsched.a:sched_roundrobin.*(.literal.nxsched_process_roundrobin .text.nxsched_process_roundrobin)
    *libsched.a:sched_reprioritizertr.*(.literal.nxsched_reprioritize_rtr .text.nxsched_reprioritize_rtr)
    *libsched.a:sched_removereadytorun.*(.literal.nxsched_remove_readytorun .text.nxsched_remove_readytorun)
    *libsched.a:sched_addreadytorun.*(.literal.nxsched_add_readytorun .text.nxsched_add_readytorun)
    *libsched.a:sched_addprioritized.*(.literal.nxsched_add_prioritized .text.nxsched_add_prioritized)
    *libsched.a:sched_mergepending.*(.literal.nxsched_merge_pending .text.nxsched_merge_pending)
    *libsched.a:sched_resumescheduler.*(.literal.nxsched_resume_scheduler .text.nxsched_resume_scheduler)

    *libc.a:sq_remfirst.*(.literal.sq_remfirst .text.sq_remfirst)


    *libarch.a:esp32s3_spi_timing.*(.literal .text .literal.* .text.*)

    *libarch.a:esp32s3_psram_quad.*(.literal .text .literal.* .text.*)

    *(.wifirxiram .wifirxiram.*)
    *(.wifi0iram .wifi0iram.*)
    *(.wifiorslpiram .wifiorslpiram.*)
    *(.wifislpiram .wifislpiram.*)
    *(.wifislprxiram .wifislprxiram.*)
    *(.phyiram .phyiram.*)
    . = ALIGN(4) + 16;
    _iram_text = ABSOLUTE(.);
  } >iram0_0_seg
  .dram0.dummy (NOLOAD) :
  {
    . = ORIGIN(dram0_0_seg) + MAX(_iram_end, _diram_i_start) - _diram_i_start;
  } >dram0_0_seg
  .dram0.bss (NOLOAD) :
  {
    . = ALIGN(8);
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
  } >dram0_0_seg
  .noinit (NOLOAD) :
  {
    . = ALIGN(4);
    *(.noinit .noinit.*)
    . = ALIGN(4);
  } >dram0_0_seg
  .dram0.data :
  {
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
    *libphy.a:(.rodata .rodata.*)
    *libarch.a:xtensa_context.*(.rodata .rodata.*)
    _edata = ABSOLUTE(.);
    . = ALIGN(4);
    _sheap = ABSOLUTE(.);
  } >dram0_0_seg
  .flash.text :
  {
    _stext = .;
    *(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.irom0.text)
    *(.fini.literal)
    *(.fini)
    *(.gnu.version)
    . += 16;
    _etext = .;
  } >default_code_seg
  .flash_rodata_dummy (NOLOAD) :
  {
    . = ALIGN(ALIGNOF(.flash.text));
    . = SIZEOF(.flash.text);
    . = ALIGN(0x10000) + 0x20;
    _rodata_reserved_start = .;
  } >default_rodata_seg
  .flash.rodata : ALIGN(0x10)
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
    . = ALIGN(4);
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
    _erodata = ABSOLUTE(.);
    _lit4_start = ABSOLUTE(.);
    *(*.lit4)
    *(.lit4.*)
    *(.gnu.linkonce.lit4.*)
    _lit4_end = ABSOLUTE(.);
    _rodata_reserved_end = ABSOLUTE(.);
    . = ALIGN(4);
  } >default_rodata_seg
  .iram0.text_end (NOLOAD) :
  {
    . += 16;
    . = ALIGN(256);
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
  .rtc.data :
  {
    *(.rtc.data)
    *(.rtc.rodata)
    . = ALIGN (4);
    _srtcheap = ABSOLUTE(.);
  } > rtc_slow_seg
}

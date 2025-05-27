PROVIDE( cache_writeback_addr = Cache_WriteBack_Addr );
MEMORY
{
  iram0_0_seg (RX) : org = (0x40370000 + 0x8000), len = (((0x403cc700 - (0x40378000 - 0x3fc88000)) - 0x3fc88000) + 0x8000 - 0x8000)
  irom0_0_seg (RX) : org = 0x42000020, len = 0x400000 - 0x20
  dram0_0_seg (RW) : org = (0x3fc88000), len = ((0x403cc700 - (0x40378000 - 0x3fc88000)) - 0x3fc88000)
  drom0_0_seg (R) : org = 0x3c000020, len = 0x400000 - 0x20
  rtc_iram_seg(RWX) : org = 0x600fe000, len = 0x2000 - 24
  rtc_data_seg(RW) : org = 0x600fe000, len = 0x2000 - 24
  rtc_reserved_seg(RW) : org = 0x600fe000 + 0x2000 - 24, len = 24
  rtc_slow_seg(RW) : org = 0x50000000 + 0,
                         len = 0x2000 - 0
  extern_ram_seg(RWX) : org = 0x3c000020 , len = 0x2000000-0x20
}
  REGION_ALIAS("default_rodata_seg", drom0_0_seg);
  REGION_ALIAS("default_code_seg", irom0_0_seg);
_ertcheap = ORIGIN(rtc_slow_seg) + LENGTH(rtc_slow_seg);

PROVIDE( rom_i2c_writereg = rom_i2c_writeReg );
PROVIDE( cache_flash_mmu_set = cache_flash_mmu_set_rom);
PROVIDE( cache_flush = Cache_Flush_rom );
PROVIDE( cache_read_disable = Cache_Read_Disable_rom );
PROVIDE( cache_read_enable = Cache_Read_Enable_rom );
MEMORY
{
  iram0_0_seg (RX) : org = 0x40080000, len = 0x20000 + 0xa000
  irom0_0_seg (RX) : org = 0x400d0020, len = 0x330000 - 0x20
  dram0_0_seg (RW) : org = 0x3ffb0000 + 0,
                         len = 0x2c200 - 0 - 0
  drom0_0_seg (R) : org = 0x3f400020, len = 0x400000 - 0x20
  rtc_iram_seg (RWX) : org = 0x400c0000, len = 0x2000
  rtc_slow_seg (RW) : org = 0x50000000 + 0,
                         len = 0x1000 - 0
  extmem_seg (RWX) : org = 0x3f800000, len = 0x400000
}
  REGION_ALIAS("default_rodata_seg", drom0_0_seg);
  REGION_ALIAS("default_code_seg", irom0_0_seg);
_sram1_iram_start = 0x400a0000;
_sram1_iram_len = ( _iram_text_end > _sram1_iram_start) ? (_iram_text_end - _sram1_iram_start) : 0;
_eheap = (_iram_text_end > _sram1_iram_start) ?
  ALIGN(0x40000000 - _sram1_iram_len - 3, 4) - 0 :
  0x40000000 - 0;
_eiramheap = (_iram_text_end > _sram1_iram_start) ? ALIGN(_iram_text_end, 4) : 0x400a0000;
_ertcheap = 0x50001fff;

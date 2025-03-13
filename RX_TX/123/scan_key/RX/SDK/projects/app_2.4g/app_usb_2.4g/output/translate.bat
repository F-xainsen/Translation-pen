.\output\BinConvert  -oad .\output\boot\bk3633_boot.bin  .\output\stack\bk3633_stack.bin  .\output\app\bk3633_app.bin -m 0x4960 -l 0x4b60 -v 0x0006 -rom_v 0x0002 -e 00000000 00000000 00000000 00000000
del .\output\app\bk3633_app_stack_oad.bin
del .\output\app\bk3633_app_merge_boot_stack.out

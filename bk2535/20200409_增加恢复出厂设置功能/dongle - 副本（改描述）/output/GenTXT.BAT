@echo on
.\output\HEXBIN.exe .\output\bk_fw.hex
.\output\bin_to_crcbin.exe .\output\bk_fw.bin 32768
del .\output\*.obj
del .\*.lst
@echo off
cd /d %cd%\..\out
copy %cd%\..\Debug\fs_control.out %cd%\fs_control.out
out2rprc.exe fs_control.out fs_control.bin
HexAIS_OMAP-L138.exe -ini NandFlash.ini -o fs_control.ais fs_control.out
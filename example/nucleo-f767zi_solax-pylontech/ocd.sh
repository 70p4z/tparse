openocd -f "interface/stlink.cfg" -c "gdb_port 9007" -c "hla_serial 066CFF555187534867203157" -f target/stm32f7x.cfg

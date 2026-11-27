# Testbench for rtl/cv_printer_uart.sv. Self contained: make && ./obj_dir/Vcv_printer_uart

all: obj_dir/Vcv_printer_uart
	./obj_dir/Vcv_printer_uart

obj_dir/Vcv_printer_uart: ../../rtl/cv_printer_uart.sv tb_main.cpp
	verilator -cc --exe --build -O2 -Wno-fatal \
	  --top-module cv_printer_uart \
	  -CFLAGS "-O2" \
	  ../../rtl/cv_printer_uart.sv tb_main.cpp

clean:
	rm -rf obj_dir

.PHONY: all clean

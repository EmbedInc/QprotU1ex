@echo off
rem
rem   Build the firmware from this source directory.
rem
setlocal
call build_fwinit

call src_aspic %srcdir% %fwname%_cmd
call src_aspic %srcdir% %fwname%_dcc
call src_aspic %srcdir% %fwname%_iic
call src_aspic %srcdir% %fwname%_init
call src_aspic %srcdir% %fwname%_intr
call src_aspic %srcdir% %fwname%_main
call src_aspic %srcdir% %fwname%_name
call src_aspic %srcdir% %fwname%_port
call src_aspic %srcdir% %fwname%_prom
call src_aspic %srcdir% %fwname%_regs
call src_aspic %srcdir% %fwname%_stack
call src_aspic %srcdir% %fwname%_strt
call src_aspic %srcdir% %fwname%_task
call src_aspic %srcdir% %fwname%_uart
call src_aspic %srcdir% %fwname%_usb
call src_aspic %srcdir% %fwname%_usb0

call src_libpic %srcdir% %fwname%

call src_aspic %srcdir% %fwname%_strt
call src_expic %srcdir% %fwname%

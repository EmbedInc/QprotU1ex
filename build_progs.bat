@echo off
rem
rem   BUILD_PROGS
rem
rem   Build the executable programs from this source directory.
rem
setlocal
call build_pasinit

call src_prog %srcdir% playwav
call src_prog %srcdir% test_u1ex
call src_prog %srcdir% u1ex_list

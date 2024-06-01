@echo off
rem
rem   Set up for building a Pascal module.
rem
call build_vars

call src_getbase
call src_getfrom stuff stuff.ins.pas
call src_getfrom pic pic.ins.pas

call src_get %srcdir% u1ex.ins.pas

make_debug debug_switches.ins.pas
call src_builddate "%srcdir%"

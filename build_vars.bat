@echo off
rem
rem   Define the variables for running builds from this source library.
rem
set srcdir=qprot
set buildname=u1ex
call treename_var "(cog)source/qprot/u1ex" sourcedir
set libname=
set fwname=u1ex
set pictype=18F2550
set picclass=PIC
set t_parms=
call treename_var "(cog)src/%srcdir%/debug_%fwname%.bat" tnam
make_debug "%tnam%"
call "%tnam%"

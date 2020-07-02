{   Program PLAYWAV filename
*
*   Quick hack to play a WAV file using a U1EX03 device.
}
program playwav;
%include '(cog)lib/base.ins.pas';
%include 'stuff.ins.pas';
%include 'u1ex.ins.pas';

const
  f_inst = 12.0e6;                     {firmware instruction clock frequency}
  max_msg_parms = 2;                   {max parameters we can pass to a message}

  f_pwm = f_inst / 127.0;              {PWM frequency}
  f_samp = f_pwm / 4.0;                {PWM output sample rate}
  t_samp = 1.0 / f_samp;               {seconds per PWM sample}

var
  fnam_in:                             {WAV file name}
    %include '(cog)lib/string_treename.ins.pas';
  name:                                {name of device to open, empty = first available}
    %include '(cog)lib/string80.ins.pas';
  conn: file_conn_t;                   {connection to the remote unit}
  wrlock: sys_sys_threadlock_t;        {lock for writing to standard output}
  thid_in: sys_sys_thread_id_t;        {ID of read input thread}
  fwver: sys_int_machine_t;            {1-N firmware version number}
  wavin: wav_in_t;                     {WAV file reading state}
  usbid: file_usbid_t;                 {USB device unique ID}
  obuf:                                {output bytes data buffer}
    %include '(cog)lib/string8192.ins.pas';
  quit: boolean;                       {TRUE when trying to exit the program}
  newline: boolean;                    {STDOUT stream is at start of new line}
  i1, i2, i3: sys_int_machine_t;       {scratch integers}
  r1: real;                            {scratch floating point}
  iname_set: boolean;                  {WAV input file name set}

  opt:                                 {upcased command line option}
    %include '(cog)lib/string_treename.ins.pas';
  parm:                                {command line option parameter}
    %include '(cog)lib/string_treename.ins.pas';
  pick: sys_int_machine_t;             {number of token picked from list}
  msg_parm:                            {parameter references for messages}
    array[1..max_msg_parms] of sys_parm_msg_t;
  stat: sys_err_t;                     {completion status code}

label
  next_opt, err_parm, parm_bad, done_opts;
{
****************************************************************************
*
*   Subroutine LOCKOUT
*
*   Acquire exclusive lock for writing to standard output.
}
procedure lockout;

begin
  sys_thread_lock_enter (wrlock);
  if not newline then writeln;         {start on a new line}
  newline := true;                     {init to STDOUT will be at start of line}
  end;
{
****************************************************************************
*
*   Subroutine UNLOCKOUT
*
*   Release exclusive lock for writing to standard output.
}
procedure unlockout;

begin
  sys_thread_lock_leave (wrlock);
  end;
{
****************************************************************************
*
*   Subroutine WHEX (B)
*
*   Write the byte value in the low 8 bits of B as two hexadecimal digits
*   to standard output.
}
procedure whex (                       {write hex byte to standard output}
  in      b: sys_int_machine_t);       {byte value in low 8 bits}
  val_param; internal;

var
  tk: string_var16_t;                  {hex string}
  stat: sys_err_t;

begin
  tk.max := size_char(tk.str);         {init local var string}

  string_f_int_max_base (              {make the hex string}
    tk,                                {output string}
    b & 255,                           {input integer}
    16,                                {radix}
    2,                                 {field width}
    [ string_fi_leadz_k,               {pad field on left with leading zeros}
      string_fi_unsig_k],              {the input integer is unsigned}
    stat);
  write (tk.str:tk.len);               {write the string to standard output}
  end;
{
****************************************************************************
*
*   Subroutine WDEC (B)
*
*   Write the byte value in the low 8 bits of B as an unsigned decimal
*   integer to standard output.  Exactly 3 characters are written with
*   leading zeros as blanks.
}
procedure wdec (                       {write byte to standard output in decimal}
  in      b: sys_int_machine_t);       {byte value in low 8 bits}
  val_param; internal;

var
  tk: string_var16_t;                  {hex string}
  stat: sys_err_t;

begin
  tk.max := size_char(tk.str);         {init local var string}

  string_f_int_max_base (              {make the hex string}
    tk,                                {output string}
    b & 255,                           {input integer}
    10,                                {radix}
    3,                                 {field width}
    [string_fi_unsig_k],               {the input integer is unsigned}
    stat);
  write (tk.str:tk.len);               {write the string to standard output}
  end;
{
****************************************************************************
*
*   Subroutine WPRT (B)
*
*   Show the byte value in the low 8 bits of B as a character, if it is
*   a valid character code.  If not, write a description of the code.
}
procedure wprt (                       {show printable character to standard output}
  in      b: sys_int_machine_t);       {byte value in low 8 bits}
  val_param; internal;

var
  c: sys_int_machine_t;                {character code}

begin
  c := b & 255;                        {extract the character code}

  case c of                            {check for a few special handling cases}
0: write ('NULL');
7: write ('^G bell');
10: write ('^J LF');
13: write ('^M CR');
17: write ('^Q Xon');
19: write ('^S Xoff');
27: write ('Esc');
32: write ('SP');
127: write ('DEL');
otherwise
    if c >= 33 then begin              {printable character ?}
      write (chr(c));                  {let system display the character directly}
      return;
      end;
    if (c >= 1) and (c <= 26) then begin {CTRL-letter ?}
      write ('^', chr(c+64));
      return;
      end;
    end;                               {end of special handling cases}
  end;
{
****************************************************************************
*
*   Subroutine THREAD_IN (ARG)
*
*   This routine is run in a separate thread.  It reads data bytes
*   from the input and writes information about the received
*   data to standard output.
}
procedure thread_in (                  {process input from the remote unit}
  in      arg: sys_int_adr_t);         {unused argument}
  val_param; internal;

var
  ibuf: array [0 .. 63] of char;       {raw input buffer}
  ibufi: sys_int_machine_t;            {index of next byte to read from IBUF}
  ibufn: sys_int_adr_t;                {number of bytes left to read from IBUF}
  b: sys_int_machine_t;                {scratch data byte value}
  i: sys_int_machine_t;                {scratch integer and loop counter}
  tk: string_var80_t;                  {scratch token}

label
  next_rsp;
{
******************************
*
*   Local function IBYTE
*
*   Return the next byte from the input stream.
}
function ibyte                         {return next byte from remote system}
  :sys_int_machine_t;                  {0-255 byte value}

var
  b: sys_int_machine_t;                {the returned byte value}
  stat: sys_err_t;                     {completion status}

label
  retry;

begin
  if quit then begin                   {trying to exit the program ?}
    sys_thread_exit;
    end;

retry:                                 {back here after reading new chunk into buffer}
  if ibufn > 0 then begin              {byte is available in local buffer ?}
    b := ord(ibuf[ibufi]);             {get the data byte to return}
    ibufi := ibufi + 1;                {advance buffer index for next time}
    ibufn := ibufn - 1;                {count one less byte left in the buffer}
    ibyte := b;                        {return the data byte}
    return;
    end;

  file_read_embusb (                   {read next chunk of data from remote device}
    conn,                              {connection to the device}
    sizeof(ibuf),                      {max amount of data allowed to read}
    ibuf,                              {input buffer to return data in}
    ibufn,                             {number of bytes actually read}
    stat);
  if quit then begin                   {trying to exit the program ?}
    sys_thread_exit;
    end;
  sys_error_abort (stat, '', '', nil, 0);
  ibufi := 0;                          {reset to fetch from start of buffer}

  goto retry;                          {back to return byte from new chunk}
  end;
{
******************************
*
*   Executable code for subroutine THREAD_IN.
}
begin
  tk.max := size_char(tk.str);         {init local var string}
  ibufn := 0;                          {init input buffer to empty}

next_rsp:                              {back here to read each new response packet}
  b := ibyte;                          {get response opcode byte}
  case b of                            {which response opcode is it ?}
{
*   NOP
}
0: begin
  end;
{
*   FWVER version
}
1: begin
  fwver := ibyte;                      {get the firmware version number}
  end;
{
*   NAME len string
}
2: begin
  tk.len := 0;                         {init name string to empty}
  b := ibyte;                          {get number of characters in the name string}
  for i := 1 to b do begin             {once for each name character}
    string_append1 (tk, chr(ibyte));
    end;
  end;
{
*   Unrecognized response opcode.
}
otherwise
    lockout;
    write ('Unrecognized response opcode: ');
    wdec (b);                          {show byte value in decimal}
    write (' ');
    whex (b);                          {show byte value in HEX}
    write ('h ');
    wprt (b);                          {show printable character, if possible}
    writeln;
    unlockout;
    end;

  goto next_rsp;                       {done with this response, back for next}
  end;
{
****************************************************************************
*
*   Start of main routine.
}
begin
{
*   Initialize our state before reading the command line options.
}
  string_cmline_init;                  {init for reading the command line}
  iname_set := false;                  {init to no WAV file specified}
{
*   Back here each new command line option.
}
next_opt:
  string_cmline_token (opt, stat);     {get next command line option name}
  if string_eos(stat) then goto done_opts; {exhausted command line ?}
  sys_error_abort (stat, 'string', 'cmline_opt_err', nil, 0);
  if opt.len <= 0 then goto next_opt;  {ignore blank command line options}
  if (opt.str[1] <> '-') then begin    {this is implicit input file name ?}
    if not iname_set then begin        {input file name not already specified ?}
      string_copy (opt, fnam_in);      {save input file name}
      iname_set := true;
      goto next_opt;
      end;
    sys_msg_parm_vstr (msg_parm[1], opt);
    sys_message_bomb ('string', 'cmline_opt_conflict', msg_parm, 1);
    end;
  string_upcase (opt);                 {make upper case for matching list}
  string_tkpick80 (opt,                {pick command line option name from list}
    '-N -WAV',
    pick);                             {number of keyword picked from list}
  case pick of                         {do routine for specific option}
{
*   -N name
}
1: begin
  string_cmline_token (name, stat);
  end;
{
*   -WAV filename
}
2: begin
  if iname_set then begin              {input file name already set ?}
    sys_msg_parm_vstr (msg_parm[1], opt);
    sys_message_bomb ('string', 'cmline_opt_conflict', msg_parm, 1);
    end;
  string_cmline_token (fnam_in, stat);
  iname_set := true;
  end;
{
*   Unrecognized command line option.
}
otherwise
    string_cmline_opt_bad;             {unrecognized command line option}
    end;                               {end of command line option case statement}

err_parm:                              {jump here on error with parameter}
  string_cmline_parm_check (stat, opt); {check for bad command line option parameter}
  goto next_opt;                       {back for next command line option}

parm_bad:                              {jump here on got illegal parameter}
  string_cmline_reuse;                 {re-read last command line token next time}
  string_cmline_token (parm, stat);    {re-read the token for the bad parameter}
  sys_msg_parm_vstr (msg_parm[1], parm);
  sys_msg_parm_vstr (msg_parm[2], opt);
  sys_message_bomb ('string', 'cmline_parm_bad', msg_parm, 2);

done_opts:                             {done with all the command line options}
{
*   All done reading the command line.
}
  usbid := file_usbid (u1ex_vid_k, u1ex_pid_k); {make combined USB dev unique ID}

  file_open_embusb (                   {open I/O connection to the device}
    usbid,                             {unique ID for our device type}
    name,                              {user-defined device name, if any}
    conn,                              {returned I/O connection}
    stat);
  sys_error_abort (stat, '', '', nil, 0);
{
*   Perform some system initialization.
}
  wav_in_open_fnam (wavin, fnam_in, stat); {open the WAV file}
  sys_error_abort (stat, '', '', nil, 0);

  sys_thread_lock_create (wrlock, stat); {create interlock for writing to STDOUT}
  sys_error_abort (stat, '', '', nil, 0);

  quit := false;                       {init to not trying to exit the program}
  newline := true;                     {STDOUT is currently at start of new line}
  fwver := 0;                          {indicate firmware version not known yet}

  sys_thread_create (                  {start thread for reading serial line input}
    addr(thread_in),                   {address of thread root routine}
    0,                                 {argument passed to thread (unused)}
    thid_in,                           {returned thread ID}
    stat);
  sys_error_abort (stat, '', '', nil, 0);

  file_write_embusb (chr(3), conn, 1, stat); {send FWVER command}
  sys_error_abort (stat, '', '', nil, 0);
  while fwver = 0 do begin             {wait until received firmware version}
    sys_thread_yield;                  {let other tasks run for a while}
    end;                               {back and check for firmware version again}
  if fwver < 3 then begin              {this firmware doesn't support HSAMP command}
    writeln ('Fimware is version ', fwver, ' and does not support WAV output.');
    sys_bomb;
    end;
{
*   Send the WAV data to the remote unit using HSAMP commands.
}
  i1 := round(wavin.tsec / t_samp) + 1; {number of samples to send}

  obuf.str[1] := chr(8);               {set HSAMP opcode}
  obuf.len := 2;                       {skip over length byte or now}

  for i2 := 1 to i1 do begin           {once for each sample}
    r1 := (i2 - 1) * t_samp;           {make time of this sample}
    r1 := wav_in_iterp_chan (wavin, r1, wav_iterp_cubic_k, -1); {-1 to +1 sample value}
    i3 := trunc(r1 * 128.0);           {make -127 to +127 value of this sample}
    i3 := min(127, max(-127, i3));
    string_append1 (obuf, chr(i3));    {add this sample to output buffer}
    if obuf.len >= 64 then begin       {max command size we want to send ?}
      obuf.str[2] := chr(obuf.len - 2); {fill in number of samples byte}
      file_write_embusb (obuf.str, conn, obuf.len, stat); {send this chunk of data}
      sys_error_abort (stat, '', '', nil, 0);
      obuf.len := 2;                   {reset to ready for next data byte}
      end;
    end;
  wav_in_close (wavin, stat);          {close the WAV file}
  sys_error_abort (stat, '', '', nil, 0);

  if obuf.len > 2 then begin           {there is unsent data in OBUF ?}
    obuf.str[2] := chr(obuf.len - 2);  {fill in number of sample bytes}
    file_write_embusb (obuf.str, conn, obuf.len, stat); {send last chunk of samples}
    sys_error_abort (stat, '', '', nil, 0);
    end;

  quit := true;                        {tell all threads to shut down}
  file_close (conn);                   {close connection to the serial line}
  end.

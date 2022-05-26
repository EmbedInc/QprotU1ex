{   Program TEST_U1EX
*
*   Program to test low level communication with a U1EX USB test platform
*   device.
}
program test_usbprog;
%include 'base.ins.pas';
%include 'stuff.ins.pas';
%include 'pic.ins.pas';
%include 'u1ex.ins.pas';

const
  max_msg_parms = 2;                   {max parameters we can pass to a message}
  nholddcc = 500;                      {max DCC packets to hold}
  tholddcc = 2.0;                      {max time to hold DCC packets}
  ndccby = 8;                          {max data bytes to save for DCC packet}
  outbuf_size = 64;                    {size of buffer for bytes to send to unit}

  nholdlast = nholddcc - 1;            {last 0-N index of saved DCC packet}
  lastdccby = ndccby - 1;              {last 0-N DCC packet data byte index}

type
  dccpack_t = record                   {data about one DCC packet}
    nbytes: sys_int_machine_t;         {number of data bytes, 0 = no packet}
    dat: array[0..lastdccby] of int8u_t; {the data bytes}
    time: sys_clock_t;                 {time the packet was received}
    end;

  dccflg_k_t = (
    dccflg_shidle_k,                   {show idle packets}
    dccflg_diff_k,                     {show any packet different from previous}
    dccflg_recv_k);                    {show packet when received without history}
  dccflg_t = set of dccflg_k_t;

  dcc_t = record                       {DCC packets state}
    flags: dccflg_t;                   {set of operation flags}
    last: sys_int_machine_t;           {0-N index of last last packet, incr for new}
    pack: array[0..nholdlast] of dccpack_t; {saved packets history list}
    end;

var
  name:                                {name of device to open, empty = first available}
    %include '(cog)lib/string80.ins.pas';
  conn: file_conn_t;                   {connection to the remote unit}
  thid_in: sys_sys_thread_id_t;        {ID of read input thread}
  fwver: sys_int_machine_t;            {1-N firmware version number}
  fwseq: sys_int_machine_t;            {firmware sequence number}
  usbid: file_usbid_t;                 {USB device unique ID}
  prompt:                              {prompt string for entering command}
    %include '(cog)lib/string4.ins.pas';
  quit: boolean;                       {TRUE when trying to exit the program}
  i1, i2: sys_int_machine_t;           {integer command parameters}
  b1: boolean;                         {boolean command parameter}
  r1: double;                          {floating point command parameter}
  showin: boolean;                     {show each individual input byte}
  showver: boolean;                    {show version and exit}
  havever: boolean;                    {firmware version has been received}
  dcc: dcc_t;                          {DCC packets history and display config}
  cmds:                                {list of interactive user commands, upper case}
    %include '(cog)lib/string8192.ins.pas';

  opt:                                 {upcased command line option}
    %include '(cog)lib/string_treename.ins.pas';
  parm:                                {command line option parameter}
    %include '(cog)lib/string_treename.ins.pas';
  tk:                                  {scratch token or string}
    %include '(cog)lib/string8192.ins.pas';
  pick: sys_int_machine_t;             {number of token picked from list}
  msg_parm:                            {parameter references for messages}
    array[1..max_msg_parms] of sys_parm_msg_t;
  stat: sys_err_t;                     {completion status code}

label
  next_opt, err_parm, parm_bad, done_opts,
  loop_cmd, badcmd, tkline, loop_tk, done_tk,
  done_cmd, err_extra, err_cmparm, leave;

%include '(cog)lib/wout_local.ins.pas';
%include '(cog)lib/nextin_local.ins.pas';
%include '(cog)lib/send_local.ins.pas';
{
********************************************************************************
*
*   Subroutine SENDALL
*
*   Send all bytes buffered for transmission to the remote system.  The output
*   buffer will be empty after this call.
}
procedure sendall;                     {send all buffered data to remote system}
  val_param; internal;

var
  stat: sys_err_t;

begin
  if outbuf.len > 0 then begin         {there is buffered data to send ?}
    file_write_embusb (outbuf.str, conn, outbuf.len, stat); {send buffer contents}
    sys_error_abort (stat, '', '', nil, 0);
    end;

  outbuf.len := 0;                     {the buffer is now definitely empty}
  end;
{
********************************************************************************
*
*   Subroutine SHOW_DCCINST (BUF, NBYTES)
*
*   Write information about a multi-function decoder DCC instruction on the
*   current line intended for display to a human.  The address has already been
*   written with no space following.  The output line will be ended by the
*   caller after this routine returns.  BUF contains a DCC "instruction" as
*   described in RP 9.2.1.  This instruction was sent to the broacast address or
*   to a 7 bit or 14 bit multi-function decoder address.  BUF contains only the
*   data bytes of the DCC packet after the address information.  NBYTES is the
*   number of bytes of this instruction.
}
procedure show_dccinst (               {show DCC instruction content}
  in      buf: univ array[0..7] of int8u_t; {the instruction bytes}
  in      nbytes: sys_int_machine_t);  {number of instruction bytes}
  val_param; internal;

var
  indnext: sys_int_machine_t;          {BUF index of next byte to return}
  nobyte: boolean;                     {set if no byte when one is requested}
  b, c, d: sys_int_machine_t;          {instruction bytes}
  i1, i2: sys_int_machine_t;           {integer parameters from instruction}
  b1, b2, b3: boolean;                 {boolean parameters from instruction}
  r1: real;                            {floating point parameters from instruction}
  tk: string_var32_t;                  {scratch string}

label
  loop_inst, done_inst, missing;
{
****************************************
*
*   Local function NEXTBYTE
*
*   Returns the next sequential byte from the input buffer, BUF.  Each call to
*   this routine returns the next byte until there is no additional byte.  When
*   there is no byte to return, the function returns 0 and sets NOBYTE to TRUE.
*
*   This function is local to SHOW_DCCINST.
}
function nextbyte                      {get next byte from the caller's buffer}
  :sys_int_machine_t;                  {the 0-255 byte value}
  val_param; internal;

begin
  nextbyte := 0;                       {init the byte value to return}
  if indnext >= nbytes then begin      {no byte to return ?}
    nobyte := true;
    return;
    end;

  nextbyte := buf[indnext];            {return the byte}
  indnext := indnext + 1;              {advance the index for the next byte}
  end;
{
****************************************
*
*   Executable code for SHOW_DCCINST.
}
begin
  tk.max := size_char(tk.str);         {init local var string}

  if nbytes < 1 then return;           {no instruction bytes ?}
  indnext := 0;                        {init BUF index of next byte to get}
  nobyte := false;                     {init to all requested bytes existed}

loop_inst:                             {back here for next instruction in packet}
  b := nextbyte;                       {get the first byte of the instruction}
  if nobyte then return;               {done all instructions in this packet ?}
  if indnext > 1 then begin            {this is not first instruction in packet ?}
    write (' /');                      {separator between instructions}
    end;

  case rshft(b, 4) of                  {which opcode is it ?}
{
****************************************
*
*   Decoder control.
}
2#0000: begin
  i1 := rshft(b, 1) & 2#111;           {extract sub-opcode}
  case i1 of                           {which sub-opcode ?}

0:  begin                              {reset}
      write (' reset');
      if (b & 1) <> 0 then begin
        write (' fact def');
        end;
      end;

1:  begin                              {factory test}
      write (' factory test');
      indnext := nbytes;               {this instruction consumes rest of packet}
      end;

2:  write (' decoder ctrl 2, reserved');

3:  begin                              {set decoder flags}
      write (' setflag');
      c := nextbyte;
      if nobyte then begin
        goto missing;
        end;
      i1 := c & 7;                     {get subaddress}
      write (' subadr ');
      if i1 = 0
        then write ('all')
        else write (i1);
      write (' ');
      i2 := rshft(c, 4);               {get opcode}
      case i2 of
0:      begin
          write ('disable CV write');
          end;
4:      begin
          write ('disable ACK');
          end;
5:      begin
          write ('activate bi-dir comm');
          end;
8:      begin
          write ('set bi-dir comm');
          end;
9:      begin
          write ('enable CV write');
          end;
15:     begin
          write ('accept 1 CV write');
          end;
otherwise
        write ('opcode ', i2);
        end;
      end;

4:  write (' decoder ctrl 4, reserved');

5:  write (' set adv adr');

6:  write (' decoder ctrl 6, reserved');

7:  write (' ACK req');

    end;                               {end of decoder control sub-opcode cases}
  end;
{
****************************************
*
*   Consist control.
}
2#0001: begin
  write (' consist ctrl');
  end;
{
****************************************
*
*   Advanced operations instruction.
}
2#0010, 2#0011: begin
  i1 := b & 2#00011111;                {get opcode}
  c := nextbyte;                       {get the data byte}
  case i1 of

2#11110: begin                         {restrict max speed}
    write (' max speed');
    if nobyte then goto missing;
    b1 := (c & 2#10000000) = 0;        {enable}
    i2 := c & 2#00011111;              {speed}
    i2 := max(0, i2 - 3);
    if not b1 then begin
      write (' disable');
      goto done_inst;
      end;
    write (' set ', i2);
    end;

2#11111: begin                         {set 126-step speed}
    write (' speed');
    if nobyte then goto missing;
    b1 := (c & 2#10000000) <> 0;       {forward}
    i2 := c & 2#01111111;              {speed field value}
    i2 := max(0, i2 - 1);              {make actual speed}
    if i2 = 0 then begin               {stop ?}
      b2 := (c & 1) <> 0;              {E stop ?}
      write (' stop');
      if b2
        then write (' off')
        else write (' normal');
      goto done_inst;
      end;
    if not b1 then i2 := -i2;          {show backwards as negative speed}
    r1 := i2 / 126;                    {make -1 to +1 relative speed}
    string_f_fp_fixed (                {make percent speed string}
      tk, r1 * 100.0, 1);
    write (' ', i2, '/126 = ', tk.str:tk.len, '%');
    end;

otherwise
    write (' ', i1);
    end;
  end;
{
****************************************
*
*   Speed and direction.
}
2#0100, 2#0101, 2#0110, 2#0111: begin
  b1 := (b & 2#00100000) <> 0;         {forwards ?}
  i1 := lshft(b & 2#01111, 1) + rshft(b & 2#10000, 4); {assemble speed number}

  if i1 <= 3 then begin                {stop ?}
    b2 := (i1 & 1) = 0;                {direction matters ?}
    b3 := (i1 & 2) <> 0;               {E stop, not coast ?}
    write (' stop ');
    if b3
      then write ('off')
      else write ('normal');
    if b2 then begin
      if b1
        then write (' fwd')
        else write (' bkw');
      end;
    goto done_inst;
    end;

  i1 := i1 - 3;                        {make actual speed magnitude}
  if not b1 then i1 := -i1;            {backwards ?}
  r1 := i1 / 28.0;                     {make -1 to +1 relative speed}
  string_f_fp_fixed (                  {make percent speed string}
    tk, r1 * 100.0, 0);
  write (' speed ', i1, '/28 = ', tk.str:tk.len, '%');
  end;
{
****************************************
*
*   Function group 1.
}
2#1000, 2#1001: begin
  write (' F1 ');
  if (b & 1) = 0
    then write ('0')
    else write ('1');
  write (', F2 ');
  if (b & 2) = 0
    then write ('0')
    else write ('1');
  write (', F3 ');
  if (b & 4) = 0
    then write ('0')
    else write ('1');
  write (', F4 ');
  if (b & 8) = 0
    then write ('0')
    else write ('1');
  write (', FL ');
  if (b & 16) = 0
    then write ('0')
    else write ('1');
  end;
{
****************************************
*
*   Function group 2.
}
2#1010, 2#1011: begin
  if (b & 2#00010000) <> 0             {get starting function number}
    then i1 := 5
    else i1 := 9;
  write (' F', i1, ' ');
  if (b & 1) = 0
    then write ('0')
    else write ('1');
  i1 := i1 + 1;
  write (' F', i1, ' ');
  if (b & 2) = 0
    then write ('0')
    else write ('1');
  i1 := i1 + 1;
  write (' F', i1, ' ');
  if (b & 4) = 0
    then write ('0')
    else write ('1');
  i1 := i1 + 1;
  write (' F', i1, ' ');
  if (b & 8) = 0
    then write ('0')
    else write ('1');
  i1 := i1 + 1;
  end;
{
****************************************
*
*   Feature expansion.
}
2#1100, 2#1101: begin
  write (' feature exp');
  end;
{
****************************************
*
*   Configuration variable access, long form.
}
2#1110: begin
  write (' CV');
  i1 := rshft(b & 2#00001100, 2);      {opcode}
  c := nextbyte;
  d := nextbyte;
  if nobyte then begin
    write ('long opc ', i1);
    goto missing;
    end;
  i2 := lshft(b & 3, 8) + c + 1;       {1-1024 CV address}
  write (' ', i2);
  case i1 of

2#01: begin                            {verify}
    write (' verify ', d);
    end;

2#11: begin                            {write}
    write (' <-- ', d);
    end;

2#10: begin                            {bit manipulation}
    i1 := d & 7;                       {0-7 bit number}
    i2 := rshft(d, 3) & 1;             {0 or 1 bit value}
    b1 := (d & 2#00010000) <> 0;       {write, not verify}
    write (' bit ', i1);
    if b1
      then write (' <-- ')
      else write (' verify ');
    write (i2);
    end;

otherwise
    write (' opc ', i1, ' dat ', d);
    end;
  end;
{
****************************************
*
*   Configuration variable access, short form.
}
2#1111: begin
  write (' CVshort');
  i1 := b & 2#00001111;                {get opcode}
  c := nextbyte;
  if nobyte then begin
    write (' opc ', i1);
    goto missing;
    end;
  case i1 of

2#0010: begin
    write (' accel (CV23) ', c);
    end;

2#0011: begin
    write (' decel (CV24) ', c);
    end;

otherwise
    write (' opc ', i1, ' data ', c);
    end;
  end;
{
****************************************
}
    end;                               {end of opcode cases}

done_inst:                             {done processing this instruction}
  goto loop_inst;                      {back for next instruction this packet}

missing:
  write (' - missing byte');
  end;
{
********************************************************************************
*
*   Subroutine SHOW_ACCINST (BUF, NBYTES)
*
*   Write information about the DCC accessory decoder instruction in BUF.
*   NBYTES is the number of instruction bytes in BUF.
*
*   Raw information about this instruction has already been written to the
*   output line.  This routine should add a description of the instruction more
*   suitable for human understanding than the raw binary.  The output line will
*   be ended by the caller after this routine returns.
*
*   The bytes in BUF are all the data bytes of the DCC packet.  The first byte
*   is guaranteed to be 10xxxxxx, which indicates this is a accessory decoder
*   packet.  The packet is guaranteed to contain at least 1 byte (NBYTES >= 1).
}
procedure show_accinst (               {show DCC accessory decoder instruction}
  in      buf: univ array[0..7] of int8u_t; {the instruction bytes}
  in      nbytes: sys_int_machine_t);  {number of instruction bytes}
  val_param; internal;

var
  indnext: sys_int_machine_t;          {BUF index of next byte to return}
  nobyte: boolean;                     {set if no byte when one is requested}
  b1, b2, b3: sys_int_machine_t;       {instruction bytes}
  adr: sys_int_machine_t;              {decoder address}
  cv: sys_int_machine_t;               {1-1024 CV number}
  i1, i2, i3: sys_int_machine_t;       {integer parameters from instruction}

label
  ext, acc_cv, missing;
{
****************************************
*
*   Local function NEXTBYTE
*
*   Returns the next sequential byte from the input buffer, BUF.  Each call to
*   this routine returns the next byte until there is no additional byte.  When
*   there is no byte to return, the function returns 0 and sets NOBYTE to TRUE.
*
*   This function is local to SHOW_ACCINST.
}
function nextbyte                      {get next byte from the caller's buffer}
  :sys_int_machine_t;                  {the 0-255 byte value}
  val_param; internal;

begin
  nextbyte := 0;                       {init the byte value to return}
  if indnext >= nbytes then begin      {no byte to return ?}
    nobyte := true;
    return;
    end;

  nextbyte := buf[indnext];            {return the byte}
  indnext := indnext + 1;              {advance the index for the next byte}
  end;
{
****************************************
*
*   Executable code for SHOW_ACCINST.
}
begin
  indnext := 0;                        {init BUF index of next byte to get}
  nobyte := false;                     {init to all requested bytes existed}

  b1 := nextbyte;                      {get the first byte of the instruction}
  b2 := nextbyte;                      {get the mandatory second byte}
  if nobyte then begin
    write ('Acc');
    goto missing;
    end;

  if (b2 & 2#10000000) = 0 then goto ext; {extended addressing being used ?}
{
*   Basic 9 bit address.
}
  adr :=                               {assemble the 9 bit address}
    (b1 & 2#00111111) !                {low 6 bits from the first byte}
    lshft(~b2 & 2#01110000, 2);        {high 3 bits from second byte}
  if adr = 511
    then write (' A-bcast')
    else write (' A', adr);

  i1 := b2 & 2#00000111;               {output number to set}

  if nbytes = 2 then begin             {not a CV instruction ?}
    i2 := rshft(b2 & 2#00001000, 3);   {0 or 1 value to set it to}
    write (' outp ', i1, ' <-- ', i2);
    if i2 = 1 then begin               {could be special turnout on/off ?}
      i3 := ((adr - 1) * 4) + rshft(i1, 1) + 1; {make turnout number}
      write (' (Turnout ', i3, ' ');
      if (i1 & 1) = 0
        then write ('reverse')
        else write ('normal');
      write (')');
      end;
    return;
    end;

  if (b2 & 2#00001111) = 0
    then write (' all outp')
    else write (' outp ', i1);
  goto acc_cv;                         {to common code for CV instruction}
{
*   Extended 11 bit address.
}
ext:
  adr :=                               {assemble the 11 bit address}
    (b1 & 2#00111111) !                {low 6 bits from the first byte}
    lshft(~b2 & 2#01110000, 2) !       {next high 3 bits from second byte}
    lshft(b2 & 2#00000110, 8);         {highest 2 bits}
  if adr = 2047
    then write ('AX-bcast')
    else write ('AX', adr);

  if nbytes <= 3 then begin            {not a CV instruction ?}
    b3 := nextbyte;
    if nobyte then goto missing;
    i1 := b3 & 2#00011111;             {get the data value}
    write (' <-- ', i1);
    return;
    end;
{
*   This is a CV instruction.  The next byte is the first of the three CV
*   instruction bytes.
}
acc_cv:
  b1 := nextbyte;                      {get the 3 CV instruction bytes}
  b2 := nextbyte;
  b3 := nextbyte;
  if nobyte then begin
    write (' CV');
    goto missing;
    end;

  cv :=                                {extract the raw 10 bit CV number}
    lshft(b1 & 2#00000011, 8) !        {most significant bits}
    b2;
  cv := cv + 1;                        {CV numbers are 1-1024}
  write (' CV ', cv);

  i1 := rshft(b1 & 2#00001100, 2);     {get the opcode}

  case i1 of
1:  begin                              {verify}
      write (' verify ', b3);
      end;
2:  begin                              {bit manipulation, data is 111CDBBB}
      i2 := b3 & 2#00000111;           {get the bit number}
      i3 := rshft(b3 & 2#00001000, 3); {get the bit value}
      write (' bit ', i2);
      if (b3 & 2#00010000) = 0
        then write (' verify ', i3)
        else write (' <-- ', i3);
      end;
3:  begin                              {write}
      write (' <-- ', b3);
      end;
otherwise
    write (' opc ', i1, ' dat ', b3);
    end;                               {end of CV opcode cases}
  return;

missing:
  write (' - missing byte');
  end;
{
********************************************************************************
*
*   Subroutine DCC_PACKET (BUF, NBYTES)
*
*   Process a newly received DCC packet.  The packet contains NBYTES bytes,
*   which are in BUF.
}
procedure dcc_packet (                 {process new received DCC packet}
  in      buf: univ array[0..7] of int8u_t; {the packet bytes}
  in      nbytes: sys_int_machine_t);  {1-N number of bytes in the packet}
  val_param; internal;

var
  now: sys_clock_t;                    {timestamp for the new packet}
  ii, jj, kk: sys_int_machine_t;       {scratch integers}
  adr: sys_int_machine_t;              {address from packet}
  idle: boolean;                       {this is a idle packet}
  dt: real;                            {time from old packet to this one}
  tk: string_var32_t;                  {scratch token}
  stat: sys_err_t;

label
  show, done_show;
{
********************
*
*   Local function SAME (I1, I2)
*
*   Returns TRUE iff the packet in the history list at index I1 is the same as
*   the packet at I2.
}
function same (                        {check for packets the same}
  in   i1, i2: sys_int_machine_t)      {history list indexes of packets to compare}
  :boolean;                            {the two packets are the same}
  val_param; internal;

var
  ind: sys_int_machine_t;              {packets byte index}

begin
  same := false;                       {init to packets do not match}
  if dcc.pack[i1].nbytes <> dcc.pack[i2].nbytes {not the same number of data bytes ?}
    then return;

  for ind := 0 to dcc.pack[i1].nbytes - 1 do begin {compare the data bytes}
    if dcc.pack[i1].dat[ind] <> dcc.pack[i2].dat[ind] {mismatch}
      then return;
    end;

  same := true;                        {the packets are the same}
  end;
{
********************
*
*   Start of DCC_PACKET executable code.
}
begin
  now := sys_clock;                    {make timestamp for this new packet}
  tk.max := size_char(tk.str);         {init local var string}
{
*   Set IDLE according to whether this is a idle packet.
}
  idle :=
    (nbytes = 2) and
    (buf[0] = 2#11111111) and
    (buf[1] = 2#00000000);

  if idle and (not (dccflg_shidle_k in dcc.flags)) {ignore this idle packet ?}
    then return;
{
*   Save this new packet as the last entry in the history list.
}
  ii := dcc.last + 1;                  {make index for this new packet}
  if ii > nholdlast then ii := 0;      {wrap back to start after end}

  jj := min(nbytes, ndccby);           {number of bytes to actually process}
  dcc.pack[ii].nbytes := jj;           {save number of bytes in the packet}
  for kk := 0 to jj-1 do begin         {save the packet bytes}
    dcc.pack[ii].dat[kk] := buf[kk];
    end;
  dcc.pack[ii].time := now;            {save the time this packet was received}
  dcc.last := ii;                      {this new packet is now the last packet}
{
*   Decide whether to show the packet.
}
  if dccflg_recv_k in dcc.flags then goto show; {show all packets as they are received ?}

  if dccflg_diff_k in dcc.flags then begin {show packet if different from previous ?}
    ii := dcc.last - 1;                {make index of previous packet}
    if ii < 0 then ii := nholdlast;    {wrap to end ?}
    if not same(dcc.last, ii) then goto show; {different ?}
    return;                            {no}
    end;
  {
  *   Show the packet only if it is "new", meaning it has not been seen in the
  *   last THOLDDCC seconds.
  }
  ii := dcc.last;                      {init index of old packet to check}
  while true do begin                  {scan backwards thru the history list}
    ii := ii - 1;                      {make index of previous packet}
    if ii < 0 then ii := nholdlast;    {wrap to end ?}
    if ii = dcc.last then exit;        {scanned thru whole list ?}
    if dcc.pack[ii].nbytes = 0 then exit; {hit end of list ?}
    dt := sys_clock_to_fp2 (           {make seconds since this packet}
      sys_clock_sub (now, dcc.pack[ii].time) );
    if dt > tholddcc then exit;        {didn't find packet within time window ?}
    if same(dcc.last, ii) then return; {found duplicate within time window ?}
    end;                               {back to check next older packet}
{
*   Show this packet.
}
show:
  lockout;
  {
  *   Show the raw bits, but don't end the output line.
  }
  write ('DCC');
  for ii := 0 to dcc.pack[dcc.last].nbytes-1 do begin {once for each packet byte}
    string_f_int_max_base (            {make binary byte value string}
      tk,                              {output string}
      dcc.pack[dcc.last].dat[ii],      {the byte value}
      2,                               {radix}
      8,                               {fixed field width}
      [ string_fi_leadz_k,             {add leading zeros to fill field}
        string_fi_unsig_k],            {the value is unsigned}
      stat);
    write (' ', tk.str:tk.len);
    end;
{
*   Show interpretation of this packet if it is a known type.
}
  if idle then begin                   {this is a idle packet ?}
    write (' Idle');
    goto done_show;
    end;

  if nbytes < 2 then goto done_show;   {illegal ?}

  adr := buf[0];                       {init to address from first byte}
  if adr = 255 then goto done_show;    {not defined other than idle ?}

  if adr = 0 then begin                {broadcast ?}
    write (' broadcast');
    show_dccinst (buf[1], nbytes-1);   {show the instruction}
    goto done_show;
    end;

  if (adr >= 1) and (adr <= 127) then begin {7 bit address ?}
    write (' S', adr);
    show_dccinst (buf[1], nbytes-1);   {show the instruction}
    goto done_show;
    end;

  if (adr >= 128) and (adr <= 191) then begin {accessory decoder range ?}
    show_accinst (buf[0], nbytes);     {show the accessory decoder instruction}
    goto done_show;
    end;

  adr := lshft(adr & 2#00111111, 8) + buf[1]; {assemble extended address}
  write (' E', adr);
  show_dccinst (buf[2], nbytes-2);     {show the instruction}

done_show:
  writeln;
  unlockout;
  end;
{
********************************************************************************
*
*   Subroutine THREAD_IN (ARG)
*
*   This routine is run in a separate thread.  It reads data bytes from the
*   input and writes information about the received data to standard output.
}
procedure thread_in (                  {process input from the remote unit}
  in      arg: sys_int_adr_t);         {unused argument}
  val_param; internal;

const
  npline = 18;                         {number of 8 bit values to write per line}
  bbuf_size = 256;                     {number of entries the scratch byte buffer can hold}

  bbuf_last = bbuf_size - 1;           {last valid index for the scratch byte buffer}

var
  ibuf: array [0 .. 63] of char;       {raw input buffer}
  ibufi: sys_int_machine_t;            {index of next byte to read from IBUF}
  ibufn: sys_int_adr_t;                {number of bytes left to read from IBUF}
  bbuf: array [0..bbuf_last] of int8u_t; {scratch byte buffer}
  bbufn: sys_int_machine_t;            {number of entries in BBUF}
  b: sys_int_machine_t;                {scratch data byte value}
  i1, i2, i3, i4, i5, i6, i7:
    sys_int_machine_t;                 {response parameters and scratch integers}
  b1, b2: boolean;                     {boolean response parameters}
  tk: string_var80_t;                  {scratch token}
  stat: sys_err_t;                     {completion status}

label
  next_rsp, done_rsp;
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
    if showin then begin               {show each individual input byte ?}
      lockout;                         {acquire exclusive lock on standard output}
      write ('Received byte: ');
      wchar (chr(b));                  {show the byte in HEX, decimal, and character}
      writeln;
      unlockout;                       {release lock on standard output}
      end;
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
*   Function GETI16U
*   This function is local to THREAD_IN.
*
*   Returns the next two input bytes interpreted as a unsigned 16 bit integer.
}
function geti16u                       {get next 2 bytes as unsigned integer}
  :sys_int_machine_t;

var
  ii: sys_int_machine_t;

begin
  ii := lshft(ibyte, 8);               {get the high byte}
  ii := ii ! ibyte;                    {get the low byte}
  geti16u := ii;
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
  lockout;
  writeln ('NOP');
  unlockout;
  end;
{
*   FWVER version
}
1: begin
  fwver := ibyte;                      {get the firmware version number}
  if fwver >= 4 then begin             {this firmware sends sequence number too ?}
    fwseq := ibyte;
    end;
  lockout;
  writeln ('Firmware U1EX ', fwver, ' seq ', fwseq);
  unlockout;
  havever := true;                     {indicate we have firmware version}
  end;
{
*   NAME len string
}
2: begin
  tk.len := 0;                         {init name string to empty}
  b := ibyte;                          {get number of characters in the name string}
  for i1 := 1 to b do begin            {once for each name character}
    string_append1 (tk, chr(ibyte));
    end;
  lockout;
  writeln ('NAME "', tk.str:tk.len, '"');
  unlockout;
  end;
{
*   SPI1 nbits dat32
}
3: begin
  i1 := ibyte;                         {get number of data bits}
  i2 := ibyte;                         {get 32 bit data value}
  i2 := i2 ! lshft(ibyte, 8);
  i2 := i2 ! lshft(ibyte, 16);
  i2 := i2 ! lshft(ibyte, 24);

  i3 := (i1 + 3) div 4;                {make number of hex digits required}
  string_f_int_max_base (              {make data value HEX string}
    tk,                                {output string}
    i2,                                {input integer}
    16,                                {number base}
    i3,                                {field width}
    [ string_fi_leadz_k,               {fill field with leading zeros}
      string_fi_unsig_k],              {treat the input number as unsigned}
    stat);
  lockout;
  writeln ('SPI1, ', i1, ' bits: ', tk.str:tk.len, 'h');
  unlockout;
  end;
{
*   SPI2 nbits dat32
}
4: begin
  i1 := ibyte;                         {get number of data bits}
  i2 := ibyte;                         {get 32 bit data value}
  i2 := i2 ! lshft(ibyte, 8);
  i2 := i2 ! lshft(ibyte, 16);
  i2 := i2 ! lshft(ibyte, 24);

  i3 := (i1 + 3) div 4;                {make number of hex digits required}
  string_f_int_max_base (              {make data value HEX string}
    tk,                                {output string}
    i2,                                {input integer}
    16,                                {number base}
    i3,                                {field width}
    [ string_fi_leadz_k,               {fill field with leading zeros}
      string_fi_unsig_k],              {treat the input number as unsigned}
    stat);
  lockout;
  writeln ('SPI2, ', i1, ' bits: ', tk.str:tk.len, 'h');
  unlockout;
  end;
{
*   IICW stat ndat
}
5: begin
  i1 := ibyte;                         {get STAT byte}
  i2 := ibyte;                         {get number of bytes actually sent}

  lockout;
  write ('IIC write completed, ', i2, ' bytes sent, ');
  if (i1 & 1) = 0
    then write ('NACK')
    else write ('ACK');
  writeln;
  unlockout;
  end;
{
*   IICR stat ndat dat ... dat
*
*   Result from IIC read sequence.
}
6: begin
  i1 := ibyte;                         {get status byte}
  i2 := ibyte;                         {get number of data bytes}
  bbufn := 0;                          {init the buffer to empty}
  for i3 := 1 to i2 do begin           {read all the data bytes}
    b := ibyte;                        {read data byte from input stream}
    if bbufn < bbuf_size then begin    {buffer has room for this byte ?}
      bbuf[bbufn] := b;                {stuff this byte into the buffer}
      bbufn := bbufn + 1;              {count one more byte in the buffer}
      end;
    end;

  lockout;
  if (i1 & 1) = 0 then begin           {ACK not received to address byte ?}
    writeln ('NACK to IIC read address byte.');
    unlockout;
    goto done_rsp;
    end;

  writeln ('IIC read, ', i2, ' data bytes:');

  i4 := min(bbufn, npline);            {make number of bytes to show}
  for i3 := 1 to i4 do begin           {write the bytes in HEX}
    b := bbuf[i3-1];
    string_f_int8h (tk, b);
    write ('  ', tk.str:tk.len);
    end;
  if i4 < bbufn then write (' ...');
  writeln;
  for i3 := 1 to i4 do begin           {write the bytes in decimal}
    b := bbuf[i3-1];
    write (' ', b:3);
    end;
  if i4 < bbufn then write (' ...');
  writeln;
  unlockout;
  end;
{
*   DCC n dat1 ... datN
*
}
7: begin
  bbufn := ibyte;                      {get number of data bytes to follow}
  for i1 := 0 to bbufn-1 do begin      {once for each data byte}
    bbuf[i1] := ibyte;                 {get this data byte, save it in the BBUF buffer}
    end;

  dcc_packet (bbuf, bbufn);            {handle this new DCC packet}
  end;
{
*   FIFO flags sizel thempt thfull put get
}
8: begin
  i1 := ibyte;                         {FLAGS}
  b1 := (i1 & 16#80) <> 0;             {EMPT}
  b2 := (i1 & 16#40) <> 0;             {FULL}
  i2 := ibyte;                         {SIZEL}
  i1 := lshft(i1 & 16#F, 8) ! i2;      {assemble SIZE in I1}
  if i1 <= 255
    then begin                         {the remaining parameters are 8 bit}
      i2 := ibyte;                     {THEMPT}
      i3 := ibyte;                     {THFULL}
      i4 := ibyte;                     {PUT}
      i5 := ibyte;                     {GET}
      i6 := ibyte;                     {N empty}
      i7 := ibyte;                     {N full}
      end
    else begin                         {the remaining parameters are 16 bit}
      i2 := geti16u;                   {THEMPT}
      i3 := geti16u;                   {THFULL}
      i4 := geti16u;                   {PUT}
      i5 := geti16u;                   {GET}
      i6 := geti16u;                   {N empty}
      i7 := geti16u;                   {N full}
      end
    ;
  {
  *   All parameters have been read, and the following variables set:
  *
  *     I1  -  FIFO size in bytes.
  *     I2  -  Empty threshold.
  *     I3  -  Full threshold.
  *     I4  -  PUT index.
  *     I5  -  GET index.
  *     i6  -  N emtpy.
  *     i7  -  N full.
  *
  *     B1  -  Empty.
  *     B2  -  Full.
  }
  lockout;
  write ('FIFO size ', i1, ', ', i7:4, ' fl ', i6:4, ' mt');
  write (', Empty ');
  if b1
    then write ('YES')
    else write (' NO');
  write (' at ', i2:4);
  write (', Full ');
  if b2
    then write ('YES')
    else write (' NO');
  write (' at ', i3:4);
  write (', PUT ', i4:4);
  write (', GET ', i5:4);
  writeln;
  unlockout;
  end;
{
*   FIFORD dat
}
9: begin
  i1 := ibyte;                         {get the data byte value}

  lockout;
  write ('FIFO read ');
  whex (i1);
  writeln (i1:4);
  unlockout;
  end;
{
*   RECVU byte
}
10: begin
  i1 := ibyte;                         {get the data byte}

  lockout;
  write ('UART recv ');
  whex (i1);
  writeln ('h ', i1:4);
  unlockout;
  end;
{
*   Unrecognized response opcode.
}
otherwise
    lockout;
    write ('Unrecognized response opcode: ');
    wchar (chr(b));                    {show the byte in HEX, decimal, and ASCII}
    writeln;
    unlockout;
    end;

done_rsp:
  goto next_rsp;                       {done with this response, back for next}
  end;
{
********************************************************************************
*
*   Start of main routine.
}
begin
  wout_init;                           {init routines that write to std out}
  send_init;                           {init routines for sending to remote unit}
{
*   Initialize our state before reading the command line options.
}
  string_cmline_init;                  {init for reading the command line}
  havever := false;                    {init to not have firmware version}
  showver := false;                    {init to not just show version and exit}
{
*   Back here each new command line option.
}
next_opt:
  string_cmline_token (opt, stat);     {get next command line option name}
  if string_eos(stat) then goto done_opts; {exhausted command line ?}
  sys_error_abort (stat, 'string', 'cmline_opt_err', nil, 0);
  string_upcase (opt);                 {make upper case for matching list}
  string_tkpick80 (opt,                {pick command line option name from list}
    '-N -VER',
    pick);                             {number of keyword picked from list}
  case pick of                         {do routine for specific option}
{
*   -N name
}
1: begin
  string_cmline_token (name, stat);
  end;
{
*   -VER
}
2: begin
  showver := true;
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
  sys_thread_lock_create (wrlock, stat); {create interlock for writing to STDOUT}
  sys_error_abort (stat, '', '', nil, 0);

  quit := false;                       {init to not trying to exit the program}
  fwver := 0;                          {indicate firmware version not known yet}
  fwseq := 0;
  dcc.flags := [];                     {show only recently new packets, ignore idle}
  dcc.last := nholdlast;               {init index of last written packet}
  for i1 := 0 to nholdlast do begin    {init each saved packet slot to empty}
    dcc.pack[i1].nbytes := 0;
    end;

  sys_thread_create (                  {start thread for reading serial line input}
    addr(thread_in),                   {address of thread root routine}
    0,                                 {argument passed to thread (unused)}
    thid_in,                           {returned thread ID}
    stat);
  sys_error_abort (stat, '', '', nil, 0);

  send_acquire;
  sendb (3);                           {send FWVER command}
  send_release;
  sendall;

  if showver then begin                {just show firmware version and exit ?}
    for i1 := 1 to 10 do begin
      sys_wait (0.100);
      if havever then goto leave;
      end;
    lockout;
    if havever then goto leave;
    writeln ('Firmware version not received within timeout.');
    sys_bomb;
    end;
{
***************************************
*
*   Process user commands.
*
*   Initialize before command processing.
}
  string_vstring (prompt, ': '(0), -1); {set command prompt string}

  string_append_token (cmds, string_v('?')); {1}
  string_append_token (cmds, string_v('HELP')); {2}
  string_append_token (cmds, string_v('Q')); {3}
  string_append_token (cmds, string_v('S')); {4}
  string_append_token (cmds, string_v('H')); {5}
  string_append_token (cmds, string_v('SHIN')); {6}
  string_append_token (cmds, string_v('LED')); {7}
  string_append_token (cmds, string_v('FWVER')); {8}
  string_append_token (cmds, string_v('NAME')); {9}
  string_append_token (cmds, string_v('SETB')); {10}
  string_append_token (cmds, string_v('SPI1')); {11}
  string_append_token (cmds, string_v('SPI2')); {12}
  string_append_token (cmds, string_v('IICW')); {13}
  string_append_token (cmds, string_v('IICR')); {14}
  string_append_token (cmds, string_v('PHF')); {15}
  string_append_token (cmds, string_v('DCC')); {16}
  string_append_token (cmds, string_v('FIFO')); {17}
  string_append_token (cmds, string_v('FIWR')); {18}
  string_append_token (cmds, string_v('FIRD')); {19}
  string_append_token (cmds, string_v('U')); {20}
  string_append_token (cmds, string_v('W')); {21}
  string_append_token (cmds, string_v('T')); {22}
  string_append_token (cmds, string_v('HL')); {23}


loop_cmd:                              {back here each new input line}
  sendall;                             {make sure all previous buffered data is sent}
  sys_wait (0.100);
  lockout;
  string_prompt (prompt);              {prompt the user for a command}
  newline := false;                    {indicate STDOUT not at start of new line}
  unlockout;

  string_readin (inbuf);               {get command from the user}
  newline := true;                     {STDOUT now at start of line}
  if inbuf.len <= 0 then goto loop_cmd; {ignore blank lines}
  p := 1;                              {init BUF parse index}
  while inbuf.str[p] = ' ' do begin    {skip over spaces before new token}
    if p >= inbuf.len then goto loop_cmd; {only blanks found, ignore line ?}
    p := p + 1;                        {skip over this blank}
    end;
  if (inbuf.str[p] = '''') or (inbuf.str[p] = '"') {quoted string ?}
    then goto tkline;                  {this line contains data tokens}
  string_token (inbuf, p, opt, stat);  {get command name token into OPT}
  if string_eos(stat) then goto loop_cmd; {ignore line if no command found}
  if sys_error(stat) then goto err_cmparm;
  string_t_int (opt, i1, stat);        {try to convert integer}
  if not sys_error (stat) then goto tkline; {this line contains only data tokens ?}
  sys_error_none (stat);
  string_upcase (opt);
  string_tkpick (opt, cmds, pick);     {pick command name from list}
  case pick of
{
*   HELP
}
1, 2: begin
  if not_eos then goto err_extra;
  lockout;
  writeln;
  writeln ('? or HELP   - Show this list of commands');
  writeln ('Q           - Quit the program');
  writeln ('S chars     - Remaining characters sent as ASCII');
  writeln ('H hex ... hex - Data bytes, tokens interpreted in hexadecimal');
  writeln ('val ... "chars" - Integer bytes or chars, chars must be quoted, "" or ''''');
  writeln ('  Integer tokens have the format: [base#]value with decimal default');
  writeln ('SHIN on|off - Show all raw input bytes from unit');
  writeln ('FWVER       - Request the firmware version');
  writeln ('NAME [name] - Set or show user-settable name');

  if fwver <= 2 then begin
    writeln ('LED on|off  - Turn the LED on or off');
    end;
  if fwver >= 4 then begin
    writeln ('SETB val    - Set the port B pins to the indicated value');
    end;
  if fwver >= 7 then
    writeln ('SPI1 nbits val - Send complete SPI frame to slave 1');
  if fwver >= 9 then
    writeln ('SPI2 nbits val - Send complete SPI frame to slave 2');
  if fwver >= 11 then
    writeln ('IICW adr dat ... dat - Send IIC write message');
  if fwver >= 12 then
    writeln ('IICR adr ndat - IIC read from address ADR, NDAT data bytes');
  if fwver >= 13 then begin
    writeln ('PHF hz      - Set 3-phase output freq,');
    writeln ('              500 max, 166 max 3-phase, 0 off');
    end;
  writeln ('DCC ALL     - Show all DCC packets');
  writeln ('DCC IDLE|NIDLE - Enable/disable showing DCC idle packets');
  writeln ('DCC DIFF    - Show DCC packets different from previous');
  writeln ('DCC NEW     - Show only new DCC packets relative to recent history');
  writeln ('FIFO        - Show FIFO current state');
  writeln ('FIWR dat [n] - Write N bytes to FIFO');
  writeln ('FIRD [n]    - Read N bytes from FIFO');
  writeln ('U dat ... dat  -  Send bytes out UART');
  writeln ('W ms        - Wait N 1 ms clock ticks');
  writeln ('HL hexcmd dat ... dat  -  Cmd and data to Hardlock');
  unlockout;
  end;
{
*   Q
}
3: begin
  if not_eos then goto err_extra;
  goto leave;
  end;
{
*   S chars
}
4: begin
  string_substr (inbuf, p, inbuf.len, parm);
  send_acquire;
  send_str (parm);
  send_release;
  end;
{
*   H hexval ... hexval
}
5: begin
  parm.len := 0;                       {init buffer of bytes to send}
  while true do begin                  {back here each new hex value}
    next_token (opt, stat);            {get the next token from the command line}
    if string_eos(stat) then exit;     {exhausted the command line ?}
    string_t_int32h (opt, i1, stat);   {convert this token to integer}
    if sys_error(stat) then goto err_cmparm;
    i1 := i1 & 255;                    {force into 8 bits}
    string_append1 (parm, chr(i1));    {one more byte to send}
    end;                               {back to get the next HEX value}
  send_acquire;
  send_str (parm);
  send_release;
  end;
{
*   SHIN on|off
*
*   Enable/disable showing all raw input bytes.
}
6: begin
  showin := next_onoff (stat);
  if sys_error(stat) then goto err_cmparm;
  if not_eos then goto err_extra;
  end;
{
*   LED on|off
}
7: begin
  b1 := next_onoff (stat);
  if sys_error(stat) then goto err_cmparm;
  if not_eos then goto err_extra;
  send_acquire;
  if b1
    then sendb (1)
    else sendb (2);
  send_release;
  end;
{
*   FWVER
}
8: begin
  if not_eos then goto err_extra;
  send_acquire;
  sendb (3);                           {FWVER opcode}
  send_release;
  end;
{
*   NAME [name]
}
9: begin
  next_token (parm, stat);             {try to get name token}
  parm.len := min(parm.len, 255);      {truncate to maximum possible name length}

  if string_eos(stat) then begin       {no command parameter supplied, get name}
    send_acquire;
    sendb (5);                         {NAMEGET opcode}
    send_release;
    goto done_cmd;
    end;

  if sys_error(stat) then goto err_cmparm;
  if not_eos then goto err_extra;

  send_acquire;
  sendb (4);                           {NAMESET opcode}
  sendb (parm.len);                    {number of name characters to follow}
  send_str (parm);                     {the name characters}
  send_release;
  end;
{
*   SETB val
}
10: begin
  i1 := next_int (0, 255, stat);
  if sys_error(stat) then goto err_cmparm;
  if not_eos then goto err_extra;

  send_acquire;
  sendb (9);                           {SETB command}
  sendb (i1);                          {data value}
  send_release;
  end;
{
*   SPI1 nbits [val]
}
11: begin
  i1 := next_int(0, 32, stat);
  if sys_error(stat) then goto err_cmparm;
  i2 := 0;                             {init data word to all zeros}
  next_token (parm, stat);
  if not string_eos(stat) then begin   {VAL parameter present ?}
    if sys_error(stat) then goto err_cmparm;
    string_t_int (parm, i2, stat);
    if sys_error(stat) then goto err_cmparm;
    if not_eos then goto err_extra;
    end;

  send_acquire;
  sendb (10);                          {SPI1 command}
  sendb (i1);                          {number of bits}
  send4 (i2);                          {32 bits word containing the data bits}
  send_release;
  end;
{
*   SPI2 nbits [val]
}
12: begin
  i1 := next_int(0, 32, stat);
  if sys_error(stat) then goto err_cmparm;
  i2 := 0;                             {init data word to all zeros}
  next_token (parm, stat);
  if not string_eos(stat) then begin   {VAL parameter present ?}
    if sys_error(stat) then goto err_cmparm;
    string_t_int (parm, i2, stat);
    if sys_error(stat) then goto err_cmparm;
    if not_eos then goto err_extra;
    end;

  send_acquire;
  sendb (11);                          {SPI2 command}
  sendb (i1);                          {number of bits}
  send4 (i2);                          {32 bits word containing the data bits}
  send_release;
  end;
{
*   IICW adr dat ... dat
}
13: begin
  i1 := next_int (0, 127, stat);       {get the IIC slave address}
  if sys_error(stat) then goto err_cmparm;

  parm.len := 0;                       {init buffer of data bytes}
  while true do begin                  {back here each new data byte}
    i2 := next_int (-128, 255, stat);  {get next data byte}
    if string_eos(stat) then exit;     {hit end of command line ?}
    if sys_error(stat) then goto err_cmparm;
    string_append1 (parm, chr(i2));    {add this data byte to end of command sequence}
    end;

  send_acquire;
  sendb (12);                          {IICW opcode}
  sendb (i2);                          {slave address}
  sendb (parm.len);                    {number of data bytes to follow}
  send_str (parm);                     {the data bytes}
  send_release;
  end;
{
*   IICR adr ndat
}
14: begin
  i1 := next_int (0, 127, stat);       {get the IIC slave address}
  if sys_error(stat) then goto err_cmparm;
  i2 := next_int (0, 255, stat);       {get the number of data bytes to read}
  if sys_error(stat) then goto err_cmparm;
  if not_eos then goto err_extra;

  send_acquire;
  sendb (13);                          {IICR opcode}
  sendb (i1);                          {slave address}
  sendb (i2);                          {number of data bytes to receive}
  send_release;
  end;
{
*   PHF hz
}
15: begin
  r1 := next_fp (stat);
  if sys_error(stat) then goto err_cmparm;
  if not_eos then goto err_extra;

  r1 := max(-500.0, min(500.0, r1));   {clip to valid range}
  r1 := r1 * 16777.216;                {scale by increment for 1 Hz}
  i1 := round(r1);                     {make 24 bit integer increment}

  send_acquire;
  sendb (14);                          {PHINC opcode}
  send3 (i1);                          {24 bit phase increment per ms}
  send_release;
  end;
{
*   DCC subcommand
*
*   Subcommands are:
*
*     ALL  -  Show all packets without any filtering.
*
*     IDLE  -  Show idle packets like other packets.
*
*     NIDLE  -  Don't show idle packets at all.
*
*     DIFF  -  Show packet whenever different from previous.
*
*     NEW  -  Show only new packets not seen recently.  This is the default.
}
16: begin
  next_token (parm, stat);
  if sys_error(stat) then goto err_cmparm;
  string_upcase (parm);
  string_tkpick80 (parm,
    'ALL IDLE NIDLE DIFF NEW',
    pick);
  case pick of
1:  begin                              {DCC ALL}
      dcc.flags := [dccflg_shidle_k, dccflg_recv_k];
      end;
2:  begin                              {DCC IDLE}
      dcc.flags := dcc.flags + [dccflg_shidle_k];
      end;
3:  begin                              {DCC NIDLE}
      dcc.flags := dcc.flags - [dccflg_shidle_k];
      end;
4:  begin                              {DCC DIFF}
      dcc.flags := dcc.flags + [dccflg_diff_k];
      dcc.flags := dcc.flags - [dccflg_recv_k];
      end;
5:  begin                              {DCC NEW}
      dcc.flags := dcc.flags - [dccflg_diff_k];
      dcc.flags := dcc.flags - [dccflg_recv_k];
      end;
otherwise
    goto badcmd;
    end;
  end;
{
*   FIFO
}
17: begin
  if not_eos then goto err_extra;

  send_acquire;
  sendb (15);                          {FIFO opcode}
  send_release;
  end;
{
*   FIWR dat [n]
}
18: begin
  i1 := next_int (0, 255, stat);       {get the data byte value}
  if sys_error(stat) then goto err_cmparm;
  i2 := next_int (1, 256, stat);       {get number of times to write data byte}
  if sys_error(stat)
    then begin                         {EOL or error}
      if not string_eos(stat) then goto err_cmparm; {hard error ?}
      i2 := 1;                         {default number of times to write the byte}
      end
    else begin                         {got N, no error}
      if not_eos then goto err_extra;
      end
    ;

  send_acquire;
  sendb (16);                          {FIFOWR opcode}
  sendb (i1);                          {data byte value}
  sendb (i2 - 1);                      {number of times to write the byte - 1}
  send_release;
  end;
{
*   FIRD [n]
}
19: begin
  i1 := next_int (1, 256, stat);       {get number of bytes to read}
  if sys_error(stat)
    then begin                         {EOL or error}
      if not string_eos(stat) then goto err_cmparm; {hard error ?}
      i1 := 1;                         {default number of bytes to read}
      end
    else begin                         {got N, no error}
      if not_eos then goto err_extra;
      end
    ;

  send_acquire;
  sendb (17);                          {FIFORD opcode}
  sendb (i1 - 1);                      {number of bytes to read - 1}
  send_release;
  end;
{
*   U dat ... dat
*
*   Write data bytes to UART.
}
20: begin
  tk.len := 0;                         {init number of bytes entered}
  while true do begin                  {back here to get each new byte}
    i1 := next_int (-128, 255, stat);  {get this byte value}
    if string_eos(stat) then exit;     {exhausted command line ?}
    if sys_error(stat) then goto err_cmparm;
    string_append1 (tk, chr(i1));      {add this byte to end of buffer}
    end;                               {back to get next byte from command line}
  if tk.len = 0 then goto done_cmd;    {no bytes to send ?}

  tk.len := min(tk.len, 256);          {clip to max bytes we can send in one command}
  send_acquire;
  sendb (18);                          {SENDU opcode}
  sendb (tk.len - 1);                  {number of data bytes - 1}
  for i1 := 1 to tk.len do begin       {send the data bytes}
    sendb (ord(tk.str[i1]));
    end;
  send_release;
  end;
{
*   W ms
}
21: begin
  i1 := next_int (1, 256, stat);       {get number of ms to wait}
  if sys_error(stat) then goto err_cmparm;
  if not_eos then goto err_extra;

  send_acquire;
  sendb (19);                          {WAITMS opcode}
  sendb (i1 - 1);                      {ms to wait - 1}
  send_release;
  end;
{
*   T
*
*   This command has no dedicated purpose, and is re-written as needed to test
*   something.
}
22: begin
  if not_eos then goto err_extra;

  send_acquire;
  sendb (18);                          {send 55h}
  sendb (0);
  sendb (16#55);

  sendb (19);                          {wait 100 ms}
  sendb (99);

  sendb (18);                          {send 55h}
  sendb (0);
  sendb (16#55);
  send_release;
  end;
{
*   HL HexCmd [dat ... dat]
*
*   Get Hardlock into command mode with command HexCmd, followed by arbitrary
*   data bytes.
}
23: begin
  i1 := next_int_hex (0, 255, stat);   {get the Hardlock command opcode}
  if sys_error(stat) then goto err_cmparm;

  tk.len := 0;                         {get any data bytes into TK}
  while true do begin                  {back here to get each new byte}
    i2 := next_int (-128, 255, stat);  {get this byte value}
    if string_eos(stat) then exit;     {exhausted command line ?}
    if sys_error(stat) then goto err_cmparm;
    string_append1 (tk, chr(i2));      {add this byte to end of buffer}
    end;                               {back to get next byte from command line}

  send_acquire;
  sendb (18);                          {SENDU opcode}
  sendb (2);                           {number of data byte - 1}
  sendb (ord('!'));                    {Hardlock wakeup sequence}
  sendb (ord('!'));
  sendb (ord('!'));

  sendb (19);                          {WAITMS opcode}
  sendb (109);                         {ms to wait - 1}

  sendb (18);                          {SENDU opcode}
  sendb (tk.len);                      {number of data byte - 1}
  sendb (i1);                          {Hardlock command opcode}
  for i2 := 1 to tk.len do begin       {send any data bytes}
    sendb (ord(tk.str[i2]));
    end;
  send_release;
  end;
{
*   Unrecognized command.
}
otherwise
badcmd:
    lockout;
    sys_msg_parm_vstr (msg_parm[1], opt);
    sys_message_parms ('string', 'err_command_bad', msg_parm, 1);
    unlockout;
    goto loop_cmd;
    end;
  goto done_cmd;                       {done handling this command}
{
*   The line contains data tokens.  Process each and add the resulting bytes to OBUF.
}
tkline:
  p := 1;                              {reset to parse position to start of line}
  tk.len := 0;                         {init buffer of byte to send}
loop_tk:                               {back here to get each new data token}
  if p > inbuf.len then goto done_tk;  {exhausted command line ?}
  while inbuf.str[p] = ' ' do begin    {skip over spaces before new token}
    if p >= inbuf.len then goto done_tk; {nothing more left on this command line ?}
    p := p + 1;                        {skip over this blank}
    end;
  if (inbuf.str[p] = '"') or (inbuf.str[p] = '''') then begin {token is a quoted string ?}
    string_token (inbuf, p, parm, stat); {get resulting string into PARM}
    if sys_error(stat) then goto err_cmparm;
    string_append (tk, parm);          {add string to bytes to send}
    goto loop_tk;                      {back to get next token}
    end;

  string_token (inbuf, p, parm, stat); {get this token into PARM}
  if sys_error(stat) then goto err_cmparm;
  string_t_int (parm, i1, stat);       {convert token to integer}
  if sys_error(stat) then goto err_cmparm;
  i1 := i1 & 255;                      {keep only the low 8 bits}
  string_append1 (tk, chr(i1));
  goto loop_tk;
done_tk:                               {done gathering data bytes into TK}
  send_acquire;
  send_str (tk);                       {send the bytes}
  send_release;

done_cmd:                              {done processing the current command}
  if sys_error(stat) then goto err_cmparm; {handle error, if any}

  if not_eos then begin                {extraneous token after command ?}
err_extra:
    lockout;
    writeln ('Too many parameters for this command.');
    unlockout;
    goto loop_cmd;
    end;

  goto loop_cmd;                       {back to process next command input line}

err_cmparm:                            {parameter error, STAT set accordingly}
  lockout;
  sys_error_print (stat, '', '', nil, 0);
  unlockout;
  goto loop_cmd;

leave:
  quit := true;                        {tell all threads to shut down}
  file_close (conn);                   {close connection to the serial line}
  end.

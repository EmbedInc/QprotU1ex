{   Program U1EX_LIST
*
*   List all U1EX devices connected to the system.
}
program u1ex_list;
%include '(cog)lib/base.ins.pas';
%include 'u1ex.ins.pas';

var
  usbid: file_usbid_t;                 {USB device unique ID}
  devs: file_usbdev_list_t;            {devices list}
  dev_p: file_usbdev_p_t;              {pointer to current devices list entry}
  stat: sys_err_t;

begin
  usbid := file_usbid (u1ex_vid_k, u1ex_pid_k); {make combined USB dev unique ID}

  file_embusb_list_get (               {get list of all connected U1EX devices}
    usbid,                             {unique ID for our device type}
    util_top_mem_context,              {parent memory context for the list}
    devs,                              {the returned list}
    stat);
  sys_error_abort (stat, '', '', nil, 0);

  if devs.n = 1
    then begin                         {exactly one device found}
      write (devs.n, ' U1EX device found');
      end
    else begin
      write (devs.n, ' U1EX devices found');
      end
    ;
  if devs.n = 0
    then writeln ('.')
    else writeln (':');

  dev_p := devs.list_p;                {init to first list entry}
  while dev_p <> nil do begin          {once for each list entry}
    writeln ('    ', dev_p^.name.str:dev_p^.name.len);
    dev_p := dev_p^.next_p;            {advance to next list entry}
    end;                               {back to do this new list entry}
  end.

cmd = Gate_Cmds.signals.values;
eng = Engine_Vars.signals.values;
t = Gate_Cmds.time;
d = iddata(eng,cmd,.005);
ssest(d,4)
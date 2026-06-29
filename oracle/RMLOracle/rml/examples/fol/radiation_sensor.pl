:- module('spec', [trace_expression/2, match/2]).
:- use_module(monitor('deep_subdict')).
match(_event, inspected(I, V)) :- deep_subdict(_{'data':V,'location':I,'topic':"inspected"}, _event).
match(_event, inspected) :- deep_subdict(_{'topic':"inspected"}, _event).
match(_event, green) :- deep_subdict(_{'data':"green",'topic':"radiationStatus"}, _event).
match(_event, orange) :- deep_subdict(_{'data':"orange",'topic':"radiationStatus"}, _event).
match(_event, red) :- deep_subdict(_{'data':"red",'topic':"radiationStatus"}, _event).
match(_event, inspect(I)) :- deep_subdict(_{'location':I,'name':"inspect",'topic':"command"}, _event).
match(_event, radiation(Min, Max)) :- deep_subdict(_{'value':V,'topic':"radiation_sensor_plugin/sensor0"}, _event), ','((V>=Min), (V<Max)).
match(_event, command) :- deep_subdict(_{'topic':"command"}, _event).
match(_event, not_command) :- not(match(_event, command)).
match(_event, r1_filter) :- match(_event, inspect(_)).
match(_event, r1_filter) :- match(_event, inspected(_, _)).
match(_event, r234_filter) :- match(_event, radiation(_, _)).
match(_event, r234_filter) :- match(_event, green).
match(_event, r234_filter) :- match(_event, orange).
match(_event, r234_filter) :- match(_event, red).
match(_event, r234_filter) :- match(_event, command).
match(_, any).
trace_expression('Main', Main) :- R1=star(var(i, (((inspect(var(i)):eps)*(inspected(var(i), 'true'):eps))\/(inspected:eps)))), R2=star((clos((plus((radiation(0, 120):eps))*(green:eps)))*optional((plus(((radiation(120, 250):eps)\/(radiation(250, 1000):eps)))*(star((not_command:eps))*(command:eps)))))), R3=star((clos((plus((radiation(120, 250):eps))*(orange:eps)))*optional((plus(((radiation(0, 120):eps)\/(radiation(250, 1000):eps)))*(star((not_command:eps))*(command:eps)))))), R4=star((clos((plus((radiation(250, 1000):eps))*(red:eps)))*optional((plus(((radiation(0, 120):eps)\/(radiation(120, 250):eps)))*(star((not_command:eps))*(command:eps)))))), Main=(((r1_filter>>R1);1)/\((r234>>((R2/\R3)/\R4));1)).

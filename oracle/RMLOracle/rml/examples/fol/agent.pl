:- module('spec', [trace_expression/2, match/2]).
:- use_module(monitor('deep_subdict')).
match(_event, current_loc(I)) :- deep_subdict(_{'data':I,'topic':"currentLoc"}, _event).
match(_event, inspected(I, V)) :- deep_subdict(_{'data':V,'location':I,'topic':"inspected"}, _event).
match(_event, inspected) :- deep_subdict(_{'topic':"inspected"}, _event).
match(_event, green) :- deep_subdict(_{'data':"green",'topic':"radiationStatus"}, _event).
match(_event, orange) :- deep_subdict(_{'data':"orange",'topic':"radiationStatus"}, _event).
match(_event, red) :- deep_subdict(_{'data':"red",'topic':"radiationStatus"}, _event).
match(_event, move(I)) :- deep_subdict(_{'location':I,'name':"move",'topic':"command"}, _event).
match(_event, move) :- deep_subdict(_{'name':"move",'topic':"command"}, _event).
match(_event, inspect(I)) :- deep_subdict(_{'location':I,'name':"inspect",'topic':"command"}, _event).
match(_event, command) :- deep_subdict(_{'topic':"command"}, _event).
match(_event, not_command) :- not(match(_event, command)).
match(_event, r1_filter) :- match(_event, inspected(_, _)).
match(_event, r1_filter) :- match(_event, green).
match(_event, r1_filter) :- match(_event, orange).
match(_event, r1_filter) :- match(_event, red).
match(_event, r1_filter) :- match(_event, move(_)).
match(_event, r1_filter) :- match(_event, current_loc(_)).
match(_event, r2_filter) :- match(_event, inspected(_, _)).
match(_event, r2_filter) :- match(_event, inspect(_)).
match(_event, r2_filter) :- match(_event, current_loc(_)).
match(_event, r3_filter) :- match(_event, green).
match(_event, r3_filter) :- match(_event, orange).
match(_event, r3_filter) :- match(_event, red).
match(_event, r3_filter) :- match(_event, move(_)).
match(_event, r4_filter) :- match(_event, inspect(_)).
match(_event, r4_filter) :- match(_event, inspected(_, _)).
match(_, any).
trace_expression('Main', Main) :- R1=(star(app(Move, [0]))*star(var(i, (clos((((plus((current_loc(var(i)):eps))|plus((inspected(var(i), 'true'):eps)))|plus((green:eps)))*app(Move, [var('i')])))*optional((plus((((orange:eps)\/(red:eps))\/(inspected(var(i), 'false'):eps)))*(star((not_command:eps))*(command:eps)))))))), R2=star(var(i, ((((inspect(var(i)):eps)*(inspected(var(i), 'true'):eps))\/((inspected(var(i), 'false'):eps)*(inspect(var(i)):eps)))|(current_loc(var(i)):eps)))), R3=star((((((orange:eps)\/(red:eps))*(move(0):eps))\/(green:eps))\/(move:eps))), Main=((((r1_filter>>R1);1)/\((r2_filter>>R2);1))/\((r3_filter>>R3);1)), Move=gen(['i'], (move(var(i)):eps)).

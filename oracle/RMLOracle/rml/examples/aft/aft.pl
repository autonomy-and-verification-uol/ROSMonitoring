:- module('spec', [trace_expression/2, match/2]).
:- use_module(monitor('deep_subdict')).
match(_event, move(Waypoint)) :- deep_subdict(_{'waypoint':Waypoint,'topic':"/command"}, _event).
match(_event, movebaseresult(Waypoint, Res)) :- deep_subdict(_{'result':Res,'waypoint':Waypoint,'topic':"/move_base/result"}, _event).
match(_event, inspect(Waypoint)) :- deep_subdict(_{'waypoint':Waypoint,'topic':"/command"}, _event).
match(_event, radiation(T1)) :- deep_subdict(_{'time':T1,'value':Value,'topic':"/radiation_sensor_plugin/sensor_0"}, _event), >=(Value, 250).
match(_event, move(NewWp, T1)) :- deep_subdict(_{'time':T2,'waypoint':NewWp,'topic':"/command"}, _event), ;((NewWp=:=0), (T2>=(T1+10))).
match(_event, movebasegoal(NewWp)) :- deep_subdict(_{'goal':MBGoal,'topic':"/move_base/goal"}, _event), =:=(NewWp, MBGoal).
match(_, any).
trace_expression('Main', Main) :- Main=var(waypoint, var(newWp, var(t1, (star((radiation(_):eps))*(((move(var(waypoint)):eps)\/(movebaseresult(var(waypoint), "success"):eps))*(star((radiation(_):eps))*((inspect(var(waypoint)):eps)*((radiation(var(t1)):eps)*(star((radiation(_):eps))*(((move(var(newWp), var(t1)):eps)\/(movebasegoal(var(newWp)):eps))*1)))))))))).

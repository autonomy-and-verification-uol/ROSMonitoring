:- module('spec', [trace_expression/2, match/2]).
:- use_module(monitor('deep_subdict')).
match(_event, dmin) :- deep_subdict(_{'distance_robot_human_operator':Dist_h_r,'robot_speed':R_speed,'human_operator_speed':H_speed,'topic':"snapshot"}, _event), >=(Dist_h_r, ((((H_speed*(0.1+0.5))+(R_speed*0.1))+0)+1.5)).
match(_, any).
trace_expression('Main', Main) :- Main=star((dmin:eps)).

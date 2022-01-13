:- module('spec', [trace_expression/2, match/2]).
:- use_module(monitor('deep_subdict')).
match(_event, msg_tell) :- deep_subdict(_{'performative':"tell"}, _event).
match(_, any).
trace_expression('Main', Main) :- Main=star((msg_tell:eps)).

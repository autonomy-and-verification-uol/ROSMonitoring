:- module('spec', [trace_expression/2, match/2]).
:- use_module(monitor('deep_subdict')).
match(_event, hello) :- deep_subdict(_{'data':"hello",'topic':"chatter"}, _event).
match(_event, world) :- deep_subdict(_{'data':"world",'topic':"madhatter"}, _event).
match(_, any).
trace_expression('Main', Main) :- Main=star(((hello:eps)\/(world:eps))).

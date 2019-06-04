:- module('spec', [trace_expression/2, match/2]).
:- use_module(monitor('deep_subdict')).
match(_event, hello) :- deep_subdict(_{'data':"hello",'topic':"chatter"}, _event).
match(_event, count) :- deep_subdict(_{'data':Val,'topic':"count"}, _event), >(Val, 100).
match(_, any).
trace_expression('Main', Main) :- Main=star(((hello:eps)\/(count:eps))).

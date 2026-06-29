:- module('spec', [trace_expression/2, match/2]).
:- use_module(monitor('deep_subdict')).
match(_event, a1) :- deep_subdict(_{'data':"a1",'topic':"topic0"}, _event).
match(_event, a2) :- deep_subdict(_{'data':"a2",'topic':"topic0"}, _event).
match(_event, b1) :- deep_subdict(_{'data':"b1",'topic':"topic1"}, _event).
match(_event, b2) :- deep_subdict(_{'data':"b2",'topic':"topic1"}, _event).
match(_event, gap) :- deep_subdict(_{'topic':"gap"}, _event).
match(_, any).

involved(8080, Evs) :- findall(Ev, (match(Ev, a1);match(Ev,a2)), Evs).
involved(8081, Evs) :- findall(Ev, (match(Ev, b1);match(Ev,b2)), Evs).

trace_expression('Main', Main) :- Main=(((b1,0.5):((b2,0.5):Main)) \/ ((b2,0.5):((b1,0.5):Main))).

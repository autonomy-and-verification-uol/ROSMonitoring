:- module('spec', [trace_expression/2, match/2]).
:- use_module(monitor('deep_subdict')).
match(_, any).
trace_expression('Main', Main) :- Main=star((any:eps)).

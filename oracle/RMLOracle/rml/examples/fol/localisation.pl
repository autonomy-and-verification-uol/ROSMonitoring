:- module('spec', [trace_expression/2, match/2]).
:- use_module(monitor('deep_subdict')).
match(_event, position(X, Y)) :- deep_subdict(_{'pose':_{'pose':_{'position':_{'y':Y,'x':X}}},'topic':"amcl_pose"}, _event).
match(_event, position(X1, Y1, X2, Y2)) :- deep_subdict(_{'pose':_{'pose':_{'position':_{'y':Y2,'x':X2}}},'topic':"amcl_pose"}, _event), ;((((X1<X2);(X1>X2));(Y1<Y2)), (Y1>Y2)).
match(_event, r1_filter) :- match(_event, position(_, _)).
match(_, any).
trace_expression('Main', Main) :- R1=star((position(_, _):eps)), Main=((r1_filter>>R1);1).

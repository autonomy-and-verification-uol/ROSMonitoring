:- module('spec', [trace_expression/2, match/2]).
:- use_module(monitor('deep_subdict')).
match(_event, move(X, Y)) :- deep_subdict(_{'y':Y,'x':X,'name':"move",'topic':"command"}, _event).
match(_event, position(X, Y)) :- deep_subdict(_{'pose':_{'pose':_{'position':_{'y':Y,'x':X}}},'topic':"amcl_pose"}, _event).
match(_event, moveBaseActionResult(Id, S)) :- deep_subdict(_{'status':_{'status':S},'header':_{'seq':Id},'topic':"move_base/result"}, _event).
match(_event, r1_filter) :- match(_event, move(_, _)).
match(_event, r1_filter) :- match(_event, position(_, _)).
match(_event, r1_filter) :- match(_event, moveBaseActionResult(_, _)).
match(_event, r2_filter) :- match(_event, move(_, _)).
match(_event, r2_filter) :- match(_event, position(_, _)).
match(_event, r2_filter) :- match(_event, moveBaseActionResult(_, _)).
match(_, any).
trace_expression('Main', Main) :- R1=star(var(x, var(y, (clos(((plus((move(var(x), var(y)):eps))|plus((position(var(x), var(y)):eps)))*(moveBaseActionResult(_, 3):eps)))*optional((plus(((move(_, _):eps)\/(position(_, _):eps)))*(moveBaseActionResult(_, _):eps))))))), R2=star(var(x, var(y, (clos(((moveBaseActionResult(_, 3):eps)*((move(var(x), var(y)):eps)|(position(var(x), var(y)):eps))))*optional((moveBaseActionResult(_, _):eps)))))), Main=(((r1_filter>>R1);1)/\((r2_filter>>R2);1)).

:- module(trace_expressions,[next/3, may_halt/1, is1/1]).

:- use_module(library(coinduction)).
:- coinductive apply_sub_trace_exp/3.

/*******************************************************************************************/
/*                              PARAMETRIC TRACE EXPRESSIONS                                                   */
/*    Aug 11, 2017: fixed bug with coinduction                                                                   */
/*    March 2, 2018: test with generic trace expressions                                                    */
/*    January, 2019: support for RML                                                                                  */
/*    May, 2019: support for const declarations                                                                  */
/*******************************************************************************************/

/* Transition rules */

% (main)
next(T, E, T1) :- next(T, E, T1, []).

%% old patch to avoid problems with json dicts, resolved by using value_string_as(atom) option
%% next(T, String, T1) :- open_string(String,Stream),json_read_dict(Stream,E,[value_string_as(atom)]), next(T, E, T1, []).

% next transition function (parametric version)
next(1,_,1,[]).  %% 1 is the universe of all traces

%% 0 is the empty set of traces, no transion rules

next(ET:T, E, T, S) :- match(E, ET, S).

next(T1\/_, E, T2, S) :- next(T1, E, T2, S),!.
next(_\/T1, E, T2, S) :- !,next(T1, E, T2, S).

next(T1|T2, E, T, S) :- next(T1, E, T3, S),!,fork(T3, T2, T).
next(T1|T2, E, T, S) :- !, next(T2, E, T3, S),fork(T1, T3, T).

next(T1*T2, E, T, S) :- next(T1, E, T3, S),!,concat(T3, T2, T).
next(T1*T2, E, T3, S) :- !,may_halt(T1),next(T2, E, T3, S).

next(T1/\T2, E, T, S) :- !,next(T1, E, T3, S1),next(T2, E, T4, S2),merge(S1, S2, S),conj(T3, T4, T).

%% version with split (really optimized?) %% important, the cut after apply_sub_trace_exp is essential to avoid divergence in case of failure due to coindcution
%% legacy clause for just one variable, no need to use a list in this case
next(var(X, T), E, T3, RetSubs) :- atom(X),!,next(T, E, T1, Subs1),split([X],Subs1,XSubs,RetSubs),apply_sub_trace_exp(XSubs,T1,T2),!,(XSubs==[]->T3=var(X,T2);T3=T2).

%% general clause
next(var(Vars, T), E, T3, RetSubs) :-
    next(T, E, T1, Subs),split(Vars,Subs,SubsVars,RetSubs),apply_sub_trace_exp(SubsVars,T1,T2),unbound(Vars,Subs,UVars),!,(UVars==[]->T3=T2;T3=var(UVars,T2)).

%% generalization for multiple variables to be implemented

%% (permanent) conditional filter
%% (ET>>T1;T2) = T1/\ET* | T2/\notET*
%% beware of this: the new symbol for conditional filter is now (_>>_;_)  (old symbol was ?/3)
%% the new symbol for if-then-else is (_?_;_) (old one was ifelse/3)  

%% remark on the syntax: >> has higher priority, hence  ET>>T1;T2 is (ET>>T1);T2

next((ET>>T1;T2), E, T, S) :- !,(match(E,ET,S1) -> next(T1,E,T3,S2),T4=T2,merge(S1, S2, S);next(T2,E,T4,S),T3=T1),filter(ET,T3,T4,T).   

%% next('?'(ET,T1,T2), E, '?'(ET,T3,T4), S) :- !,match(E,ET,S1) -> next(T1,E,T3,S2),T4=T2,merge(S1, S2, S);next(T2,E,T4,S),T3=T1.       

%% (permanent) filter
%% ET>>T1 = T1/\ET* | notET*

next(ET>>T, E, T2, S) :- !,(match(E, ET, S1) -> next(T, E, T1, S2),merge(S1, S2, S);T1=T,S=[]),filter(ET,T1,T2).


%% temporary conditional filter
%% ET>T1;T2 = (eps\/notET*/\T2) * (eps\/ET:1)/\T1
%% warning: to be tested yet

%% remark on the syntax: > has higher priority, hence  ET>T1;T2 is (ET>T1);T2

next((ET>T1;T2), E, T, S) :- !,match(E,ET,S1) -> next(T1,E,T,S2),merge(S1, S2, S);next(T2,E,T3,S),temp_filter(ET,T1,T3,T).   


%% temporary filter
%% ET>T = (eps\/notET*) * (eps\/ET:1)/\T

next((ET>T), E, T1, S) :- !,match(E, ET, S1) -> next(T, E, T1, S2),merge(S1, S2, S);S=[],temp_filter(ET,T,T1).


%% proposal for generics

%% legacy clause for just one variable, no need to use a list in this case
next(app(gen(X,T1),Arg), E, T3, S) :- atom(X),!,eval(Arg,Val),apply_sub_trace_exp([X=Val], T1, T2),!,next(T2, E, T3, S). %% agaian here the cut after apply_sub_trace_exp is essential to avoid divergence in case of failure due to coinduction

%% general clause 
next(app(gen(Vars,T1),Args), E, T3, S) :- eval_exps(Vars,Args,Sub),apply_sub_trace_exp(Sub,T1,T2),!,next(T2, E, T3, S).%% agaian here the cut after apply_sub_trace_exp is essential to avoid divergence in case of failure due to coinduction

%% proposal for guarded trace expressions 
%% comment: do we really need to return a substitution with solve? According to the ecoop19 calculus guards should be only ground
%% this should be more in line with the ecoop19/oopsla19 calculus
next(guarded(P,T1,T2),E,T,S) :- !,P -> next(T1,E,T,S);next(T2,E,T,S).

%%next(guarded(P,T1,T2),E,T,S) :- !,solve(P,S1) -> next(T1,E,T,S2), merge(S1, S2, S);next(T2,E,T,S).

%% to be tested
%% proposal for (_?_;_), similar to previous ifelse/3 (ifelse(ET,T1,T2)=(ET?ET:T1;T2))
%% ET?T1;T2 = (eps\/ET:1)/\T1 \/ (eps\/notET:1) /\ T2

% obsolete, and doesn't work on version 8
%next((ET?T1;T2), E, T, S) :- !,
%	match(E, ET, S1) -> next(T1, E, T, S2), merge(S1, S2, S) ; next(T2, E, T, S).

%% proposal for prefix closure

next(clos(T1), E, T3, S) :- !,next(T1, E, T2, S),prefix_clos(T2,T3).

% regex-like operators
next(star(T1), E, T2*star(T1), S) :- !, next(T1, E, T2, S).
next(plus(T1), E, T2*star(T1), S) :- !, next(T1, E, T2, S).
next(optional(T1), E, T2, S) :- !, next(T1, E, T2, S).

%% proposal for the with operator, to be tested
%% see the comment on solve/2 for guarded expressions

next(with(ET,T,G), E, T, S) :- !,match(E, ET, S),apply_sub_pred(S,G,G2),G2.

%% proposal for constant declarations
next(const(Vars, Exps, T1), E, T3, RetSubs) :-
    eval_exps(Vars,Exps,Subs),apply_sub_trace_exp(Subs,T1,T2),!,next(T2,E,T3,RetSubs). %% cut after apply_sub_trace_exp is essential to avoid divergence in case of failure due to coinduction


%% eval predicates for arguments of generics: for the moment only number/boolean expressions, strings and atoms are supported
num_exp(Exp) :- Exp=..[Op|_],memberchk(Op,[+,-,/,*]).

eval(Exp,Exp) :- (atom(Exp);number(Exp);string(Exp)),!.
eval(Exp,Val) :- num_exp(Exp),!,Val is Exp.
eval(Exp,Val) :- Exp -> Val=true;Val=false. %% assumes it is 		 

%% evaluates multiple expressions (to be read: arguments of generics) and assigns values to multiple variables (to be read: parameters of generics)
%% and computes a corresponding substitution
eval_exps(Vars,Exps,Sub) :- acc_eval_exps(Vars,Exps,[],Sub). 

acc_eval_exps([],[],Sub,Sub).
acc_eval_exps([X|Vars],[Exp|Exps],Acc,Sub) :- eval(Exp,Val),acc_eval_exps(Vars,Exps,[X=Val|Acc],Sub).

%% match predicate
    
%% match(E,ET,S) :- copy_term(ET,FreshET),match(E,FreshET),unifiable(ET,FreshET,S). %% old version
match(E,ET,Subs) :- copy_term_with_vars(ET,FreshET,Subs), match(E,FreshET). %%,write('matching '),write(E),write(' with '),writeln(FreshET),match(E,FreshET),write('matched ').
%% debugging version
%%match(E,ET,Subs) :- copy_term_with_vars(ET,FreshET,Subs),write('matching '),write(E),write(' with '),write(FreshET),(match(E,FreshET)->true;nl,false),writeln('... matched\n').

%% solve predicate
%% solve(P,Subs) :- !,copy_term_with_vars(P,[],FreshP,Subs),FreshP.
solve(P,Subs) :- !,copy_term_with_vars(P,FreshP,Subs),FreshP.

may_halt(1) :- !.
may_halt(eps) :- !.
may_halt(T1\/T2) :- (may_halt(T1), !; may_halt(T2)).
may_halt(T1|T2) :- !, may_halt(T1), may_halt(T2).
may_halt(T1*T2) :- !, may_halt(T1), may_halt(T2).
may_halt(T1/\T2) :- !, may_halt(T1), may_halt(T2).
may_halt(var(_, T)) :- !, may_halt(T).
may_halt((_>>T1;T2)) :- !, may_halt(T1), may_halt(T2).
may_halt(_>>T) :- !, may_halt(T).
%% ET>T = (eps\/notET*) * (eps\/ET:eps)/\T
may_halt((_>T)) :- !, may_halt(T).
%% ET>T1;T2 = (eps\/notET*/\T2) * (eps\/ET:1)/\T1
may_halt((_>T;_)) :- !, may_halt(T).


%% proposal for generics
%may_halt(app(gen(X,T1),Arg)) :- !, Val is Arg, apply_sub_trace_exp([X=Val],T1,T2),!,may_halt(T2). %% usual comment for the cut after apply_sub_trace_exp   

%% legacy clause for just one variable, no need to use a list in this case
may_halt(app(gen(X,T1),Arg)) :- atom(X),!,eval(Arg,Val),apply_sub_trace_exp([X=Val],T1,T2),!,may_halt(T2). %% usual comment for the cut after apply_sub_trace_exp   

%% generic clause
may_halt(app(gen(Vars,T1),Args)) :- eval_exps(Vars,Args,Subs),apply_sub_trace_exp(Subs,T1,T2),!,may_halt(T2). %% usual comment for the cut after apply_sub_trace_exp

%% proposal for guarded trace expressions
%% this should be more in line with the ecoop19/oopsla19 calculus
may_halt(guarded(P,T1,T2)) :- !,P->may_halt(T1);may_halt(T2).

%% may_halt(guarded(P,T1,T2)) :- !,solve(P,_)->may_halt(T1);may_halt(T2).

%% proposal for if-then-else
% may_halt never holds?

%% proposal for prefix closure

may_halt(clos(_)).

% regex-like operators
may_halt(star(_)).
may_halt(plus(T)) :- !, may_halt(T).
may_halt(optional(_)).

%% proposal for the with operator, to be tested

%% the with operator can never halt

%% proposal for constant declarations
may_halt(const(Vars, Exps, T)) :- eval_exps(Vars,Exps,Subs),apply_sub_trace_exp(Subs,T,T2),!,may_halt(T2). %% cut after apply_sub_trace_exp is essential to avoid divergence in case of failure due to coinduction

% see if trace expression is equivalent to 1 (only sound, not complete)
is1(1) :- !.
% eps is not 1
is1(T1 \/ T2) :- is1(T1) ; is1(T2).
is1(T1 | T2) :- is1(T1) ; is1(T2).
is1(T1 * T2) :- is1(T1) ; (may_halt(T1), is1(T2)).
is1(T1 /\ T2) :- is1(T1), is1(T2).
is1(var(_, T)) :- is1(T).
is1((_ >> T1 ; T2)) :- is1(T1), is1(T2).
is1(_ >> T) :- is1(T).
is1(_ > T1 ; T2) :- is1(T1), is1(T2).
is1(_ > T) :- is1(T).
is1(app(gen(_, T), _)) :- is1(T).
is1(guarded(_, T1, T2)) :- is1(T1, T2).
is1(clos(T)) :- is1(T).
is1(star(T)) :- is1(T).
is1(plus(T)) :- is1(T).
is1(optional(T)) :- is1(T).

    
%%% optimizations
fork(0,0,0) :- !.
fork(0,eps,0) :- !.
fork(eps,0,0) :- !.
fork(1,T,T) :- !.
fork(T,1,T) :- !.
fork(eps, T, T) :- !.
fork(T, eps, T) :- !.
fork((T1l|T1r), T2, (T1l|(T1r|T2))) :- !.
fork(T1, T2, (T1|T2)).

concat(0, _, 0) :- !.
concat(1, _, 1) :- !.
concat(eps, T, T) :- !.
concat(T, eps, T) :- !.
concat((T1l*T1r), T2, T1l*(T1r*T2)) :- !.
concat(T1, T2, T1*T2).

conj(eps/\eps, eps) :- !.
conj(1,T,T) :- !.
conj(1,T,T) :- !.
conj((T1l/\T1r), T2, T1l/\(T1r/\T2)) :- !.
conj(T1, T2, T1/\T2).

%% optimization for filters
%% to be done: optimizations with special predefined event types any and none

%% conditional (permanent) filter
%% (ET>>T1;T2) = T1/\ET* | T2/\notET*

filter(_,1,1,1) :- !.
filter(ET,T,1,ET>>T) :- !.
filter(ET,T1,T2,(ET>>T1;T2)).
	
%% (permanent) filter
%% (ET>>T1) = (ET>>T1;1) = T1/\ET* | 1/\notET* = T1/\ET* | notET* 

filter(_,1,1) :- !.
filter(ET,T,ET>>T).

%% conditional temporary filter
%% ET>T1;T2 = (eps\/notET*/\T2) * (eps\/ET:1)/\T1

temp_filter(_,1,1,1) :- !.
temp_filter(ET,T,1,(ET>T)) :- !.
temp_filter(ET,T1,T2,(ET>T1;T2)).

%% temporary filter
%% ET>T = (eps\/notET*) * (eps\/ET:1)/\T

temp_filter(_,1,1) :- !.
temp_filter(ET,T,(ET>T)).

%% prefix closure

prefix_clos(1,1) :- !.
prefix_clos(T,clos(T)).

%%% to be done: optimizations for filter/3 corresponding to _>>_;_

%%% substitutions

% split into a singleton/empty substitution and the rest
%% split(_,[],[],[]).
%% split(X,[Y=V|S],[Y=V],S) :- X==Y,!.
%% split(X,[A|S],S1,[A|S2]) :- split(X,S,S1,S2).

% generalized split 
split(_,[],[],[]) :- !.

split(Vs,S,S_in,S_out) :- acc_split(Vs,S,[],S_in,[],S_out). %% Vs: list of distinct variables, S: initial substitution, S_in: part of S including vars in Vs, S_out=S\S_in

%% auxiliary predicate with accumulators
acc_split(_,[],S_in,S_in,S_out,S_out).
acc_split(Vs,[X=V|S],Acc_in,S_in,Acc_out,S_out) :- %% memberchk should work, substitutions should be always ground
    memberchk(X,Vs) -> acc_split(Vs,S,[X=V|Acc_in],S_in,Acc_out,S_out);acc_split(Vs,S,Acc_in,S_in,[X=V|Acc_out],S_out). 

% auxiliary predicate to check whether substitution S maps variable X into value V 
apply(S,X,V) :- memberchk(X=V,S). %% memberchk should work, substitutions should be always ground

% auxiliary predicate to check whether a variable is in the domain of a substitution
% in_dom(X,S) :- apply(S,X,_). %% superseded by split

% unbound vars
unbound(Vars,[],Vars).
unbound(Vars,[X=_|Subs],Vars3) :- delete(Vars,X,Vars2),!,unbound(Vars2,Subs,Vars3).

% substitution application generalized to all (finite) substitutions, not just singleton and empty substitutions 
apply_sub_trace_exp([],T,T) :- !.  %% optimization
apply_sub_trace_exp(_,1,1) :- !.
apply_sub_trace_exp(_,0,0) :- !.
apply_sub_trace_exp(_,eps,eps) :- !.
apply_sub_trace_exp(S, ET1:T1, ET2:T2) :- !,apply_sub_event_type(S,ET1,ET2),apply_sub_trace_exp(S,T1,T2).
apply_sub_trace_exp(S,T1\/T2,T3\/T4) :- !,apply_sub_trace_exp(S,T1,T3),apply_sub_trace_exp(S,T2,T4).
apply_sub_trace_exp(S,T1|T2,T3|T4) :- !,apply_sub_trace_exp(S,T1,T3),apply_sub_trace_exp(S,T2,T4).
apply_sub_trace_exp(S,T1*T2,T3*T4) :- !,apply_sub_trace_exp(S,T1,T3),apply_sub_trace_exp(S,T2,T4).
apply_sub_trace_exp(S,T1/\T2,T3/\T4) :- !,apply_sub_trace_exp(S,T1,T3),apply_sub_trace_exp(S,T2,T4).
%% legacy clause for just one variable, no need to use a list in this case
apply_sub_trace_exp(S,var(X, T1),var(X, T2)) :- atom(X),!,split([X],S,_Sx,Srest),apply_sub_trace_exp(Srest,T1,T2).
%% general clause 
apply_sub_trace_exp(S,var(Vars, T1),var(Vars, T2)) :- split(Vars,S,_Svars,Srest),apply_sub_trace_exp(Srest,T1,T2).
apply_sub_trace_exp(S,(ET1>>T1;T2),(ET2>>T3;T4)) :- !,apply_sub_event_type(S,ET1,ET2),apply_sub_trace_exp(S,T1,T3),apply_sub_trace_exp(S,T2,T4).
apply_sub_trace_exp(S,ET1>>T1,ET2>>T2) :- !,apply_sub_event_type(S,ET1,ET2),apply_sub_trace_exp(S,T1,T2).
apply_sub_trace_exp(S,(ET1>T1;T2),(ET2>T3;T4)) :- !,apply_sub_event_type(S,ET1,ET2),apply_sub_trace_exp(S,T1,T3),apply_sub_trace_exp(S,T2,T4).
apply_sub_trace_exp(S,(ET1>T1),(ET2>T2)) :- !,apply_sub_event_type(S,ET1,ET2),apply_sub_trace_exp(S,T1,T2).

%% proposal for generics

% apply_sub_trace_exp([Y=V],app(gen(X,T1),Arg1),app(gen(X,T2),Arg2)) :- !,apply_sub_arg([Y=V],Arg1,Arg2),(Y==X -> T2=T1;apply_sub_trace_exp([Y=V],T1,T2)).

%% legacy clause for just one variable, no need to use a list in this case
apply_sub_trace_exp(S,app(gen(X,T1),Arg1),app(gen(X,T2),Arg2)) :-
    atom(X),!,apply_sub_arg(S,Arg1,Arg2),split([X],S,_Sx,Srest),apply_sub_trace_exp(Srest,T1,T2).

%% generic clause
apply_sub_trace_exp(S,app(gen(Vars,T1),Args1),app(gen(Vars,T2),Args2)) :-
    apply_sub_arg(S,Args1,Args2),split(Vars,S,_Svars,Srest),apply_sub_trace_exp(Srest,T1,T2).

%% proposal for guarded trace expressions

apply_sub_trace_exp(S,guarded(P1,T1,T2),guarded(P2,T3,T4)) :- !,apply_sub_pred(S,P1,P2),apply_sub_trace_exp(S,T1,T3),apply_sub_trace_exp(S,T2,T4).

% obsolete, and doesn't work on version 8
%% proposal for if-then-else
%apply_sub_trace_exp(S, (ET? T1 ; T2), (ETs ? T1s ; T2s)) :-
%	!,
%	apply_sub_event_type(S, ET, ETs),
%	apply_sub_trace_exp(S, T1, T1s),
%	apply_sub_trace_exp(S, T2, T2s).

%% proposal for prefix closure
apply_sub_trace_exp(S, clos(T1), clos(T2)) :- !,apply_sub_trace_exp(S,T1,T2).

% regex-like operators
apply_sub_trace_exp(S, star(T1), star(T2)) :- !, apply_sub_trace_exp(S, T1, T2).
apply_sub_trace_exp(S, plus(T1), plus(T2)) :- !, apply_sub_trace_exp(S, T1, T2).
apply_sub_trace_exp(S, optional(T1), optional(T2)) :- !, apply_sub_trace_exp(S, T1, T2).

%% proposal for the with operator, to be tested

apply_sub_trace_exp(S,with(ET,T,G),with(ET2,T2,G2)) :- !, apply_sub_event_type(S, ET, ET2), apply_sub_trace_exp(S, T, T2), apply_sub_pred(S,G,G2). 

%% proposal for constant declarations
apply_sub_trace_exp(S,const(Vars, Exps1, T1),const(Vars, Exps2, T2)) :- apply_sub_arg(S,Exps1,Exps2),split(Vars,S,_Svars,Srest),apply_sub_trace_exp(Srest,T1,T2).

% substitution inside event types
apply_sub_event_type([],ET,ET) :- !.
%%apply_sub_event_type([X=V],var(Y),ET) :- !,(Y==X -> ET=V;ET=var(Y)).
apply_sub_event_type(S,var(X),ET) :- !,(apply(S,X,V) -> ET=V;ET=var(X)).
apply_sub_event_type(S,ET1,ET2) :- !,ET1=..[F|Args1],apply_sub_event_type_list(S,Args1,Args2),ET2=..[F|Args2].

apply_sub_event_type_list(_,[],[]) :- !.
apply_sub_event_type_list(S,[ET1|ETL1],[ET2|ETL2]) :- !,apply_sub_event_type(S,ET1,ET2),apply_sub_event_type_list(S,ETL1,ETL2).

%% proposal for generics
%% substitution inside generic arguments

apply_sub_arg(S,Arg1,Arg2) :- !,apply_sub_event_type(S,Arg1,Arg2).

%% proposal for guarded trace expressions
%% substitution inside guard predicates

apply_sub_pred(S,P1,P2) :- !,apply_sub_event_type(S,P1,P2).

% add: auxiliary predicate for merge
% add a new association to the substitution, no addition is performed if the association is already in the substitution,
% the predicate fails if the association is not compatible with the substitution
add(A, [], [A]).
add(X=V1, [Y=V2|S], [X=V1|S]) :- X==Y,!,V1==V2.
add(A1, [A2|S], [A2|S1]) :- add(A1, S, S1).

% merge two substitutions; fails if they do not agree on the shared variables 
merge([], L, L).
merge([H|T], L, L2) :-
  add(H, L, L1),
  merge(T, L1, L2), !.

% lookup: auxiliary predicate for copy_term_with_vars
% lookup(X,Subs,V,Subs') looks up variable name X in Subs
% if found returns the corresponding associated Prolog variable V and Subs'=Subs
% otherwise returns a fresh Prolog variable V, Subs' is obtained from Subs by adding the association X=V 

lookup(X,[],Y,[X=Y]).
lookup(X,[X=V|Subs],V,[X=V|Subs]) :- !.
lookup(X,[Y=V1|Subs1],V2,[Y=V1|Subs2]) :- lookup(X,Subs1,V2,Subs2).

% copy_term_with_vars
% needed to properly manage coinduction in the presence of binder variables (they cannot be ordinary Prolog variables!)
% variables in event types are represented with the syntax "var(name)" 
% Subs is the returned list mapping variable names into corresponding Prolog variables

copy_term_with_vars(ET,FreshET,Subs) :- copy_term_with_vars(ET,[],FreshET,Subs).

copy_term_with_vars(var(X),Subs1,Var,Subs2) :- lookup(X,Subs1,Var,Subs2),!.
copy_term_with_vars(ET1,Subs1,ET2,Subs2) :- ET1=..[F|Args1],copy_term_with_vars_list(Args1,Subs1,Args2,Subs2),ET2=..[F|Args2].

copy_term_with_vars_list([],Subs,[],Subs).
copy_term_with_vars_list([ET1|ETL1],Subs1,[ET2|ETL2],Subs3) :- copy_term_with_vars(ET1,Subs1,ET2,Subs2),copy_term_with_vars_list(ETL1,Subs2,ETL2,Subs3).


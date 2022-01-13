:- module(trace_expressions,[partial_belongs/4, probabilize/2, may_halt/1, sure_gaps/2, extract_gaps/3, update_paths/5]).

:- use_module(library(assoc)).
:- use_module(library(coinduction)).

:- dynamic involved/2.
:- dynamic count/1.

% sum elements in a list
list_sum([Item], Item) :- !.
list_sum([Item1,Item2 | Tail], Total) :-
    Item is Item1+Item2,
    list_sum([Item|Tail], Total).

make_unions([T], T) :- !.
make_unions([T|Ts], T\/Union) :-
  make_unions(Ts, Union).

probabilize(T, ProbT) :-
  empty_assoc(Assoc),
  probabilize(Assoc, T, ProbT).
probabilize(Assoc, T, ProbT) :-
  get_assoc(T, Assoc, ProbT), !.
probabilize(Assoc, T, ProbT) :-
  put_assoc(T, Assoc, ProbT, Assoc1),
  findall((Ev,T1), next(T, Ev, T1), T1s),
  length(T1s, N),
  (N = 0 -> ProbT = epsilon;
  ProbEach is 1/N,
  bagof(
    ((Ev, ProbEach):ProbT1),
    T1^(
      member((Ev,T1), T1s),
      probabilize(Assoc1, T1, ProbT1)
    ),
    List
  ),
  make_unions(List, ProbT)).

test_prob(ProbT, CTProbT) :-
  empty_assoc(Assoc),
  test_prob(Assoc, ProbT, CTProbT).
test_prob(Assoc, ProbT, CTProbT) :-
  get_assoc(ProbT, Assoc, CTProbT), !.
test_prob(Assoc, ProbT, CTProbT) :-
  put_assoc(ProbT, Assoc, CTProbT, Assoc1),
  findall(((Ev,P),ProbT1), (next_p(ProbT, Ev, (ProbT1,P)), Ev \= gap(_)), T1s),
  length(T1s, N),
  (N = 0 -> CTProbT = epsilon;
  findall(P, member(((_,P),_), T1s), Ps),
  list_sum(Ps, Sum),
  (Sum = 1; Sum = 1.0),
  bagof(
    ((Ev, P):CTProbT1),
    ProbT1^(
      member(((Ev,P),ProbT1), T1s),
      test_prob(Assoc1, ProbT1, CTProbT1)
    ),
    List
  ),
  length(List, N),
  make_unions(List, CTProbT)).



may_halt(T) :-
  empty_assoc(Assoc),
  may_halt(T, Assoc).
may_halt(T, Assoc) :-
  get_assoc(T, Assoc, _), !,
  fail.
may_halt(epsilon, _) :- !.
may_halt(T1\/T2, Assoc) :-
  put_assoc(T1\/T2, Assoc, _, Assoc1),
  (may_halt(T1, Assoc1), !; may_halt(T2, Assoc1)).
may_halt(T1|T2, Assoc) :- !,
  put_assoc(T1|T2, Assoc, _, Assoc1),
  may_halt(T1, Assoc1), may_halt(T2, Assoc1).
may_halt(T1*T2, Assoc) :- !,
put_assoc(T1*T2, Assoc, _, Assoc1),
may_halt(T1, Assoc1), may_halt(T2, Assoc1).
may_halt(T1/\T2, Assoc) :- !,
put_assoc(T1/\T2, Assoc, _, Assoc1),
may_halt(T1, Assoc1), may_halt(T2, Assoc1).
may_halt(ET>>T, Assoc) :- !,
  put_assoc(ET>>T, Assoc, _, Assoc1),
  may_halt(T, Assoc1).
may_halt(var(X, T), Assoc) :- !,
  put_assoc(var(X, T), Assoc, _, Assoc1),
  may_halt(T, Assoc1).

fork(epsilon, T, T) :- !.
fork(T, epsilon, T) :- !.
fork((T1l|T1r), T2, (T1l|(T1r|T2))) :- !.
fork(T1, T2, (T1|T2)).

concat(epsilon, T, T) :- !.
concat(T, epsilon, T) :- !.
concat((T1l*T1r), T2, T1l*(T1r*T2)) :- !.
concat(T1, T2, T1*T2).

conj(epsilon, _, epsilon) :- !.
conj(_, epsilon, epsilon) :- !.
conj((T1l/\T1r), T2, T1l/\(T1r/\T2)) :- !.
conj(T1, T2, T1/\T2).

next_p((ET, Prob):T, gap(Ev), (T, Prob)) :-
  match(Ev, ET).
next_p(ET:T, gap(Ev), (T, 1)) :-
  ET \= (_,_),
  match(Ev, ET).
next_p((ET, Prob):T, Ev, (T, Prob)) :- % it depends on how it will be generated from the HMM
  var(Ev) -> match(Ev, ET); Ev \= gap(_), match(Ev, ET).
next_p(ET:T, Ev, (T, 1)) :-
  ET \= (_,_),
  (var(Ev) -> match(Ev, ET);   Ev \= gap(_), match(Ev, ET)).
next_p(T1\/_, Ev, (T2, Prob)) :-
  next_p(T1, Ev, (T2, Prob)).
next_p(_\/T1, Ev, (T2, Prob)) :-
  !, next_p(T1, Ev, (T2, Prob)).
next_p(T1|T2, Ev, (T, Prob)) :-
  next_p(T1, Ev, (T3, Prob)),
  fork(T3, T2, T).
next_p(T1|T2, Ev, (T, Prob)) :-
  !, next_p(T2, Ev, (T3, Prob)),
  fork(T1, T3, T).
next_p(T1*T2, Ev, (T, Prob)) :-
  next_p(T1, Ev, (T3, Prob)),
  concat(T3, T2, T).
next_p(T1*T2, Ev, (T3, Prob)) :-
  !, may_halt(T1),
  next_p(T2, Ev, (T3, Prob)).
next_p(T1/\T2, Ev, (T, Prob)) :-
  !, next_p(T1, Ev, (T3, Prob1)),
  next_p(T2, Ev, (T4, Prob2)),
  Prob is min(Prob1, Prob2),
  conj(T3, T4, T).

next_p((T1, Prob1), Ev, Ev1, (T2, Prob2)) :-
  !,
  (
    (may_be_none(T1, Ev),
      (Ev1 = gap(none), T2 = T1, probAgent(ProbAgent), Prob2 is Prob1 * ProbAgent));
    (next_p(T1, Ev, (T2, Prob3)), Prob2 is Prob1 * Prob3, Ev1 = Ev)
  ).

  %((Ev = gap(none), T2 = T1, probAgent(ProbAgent), Prob2 is Prob1 * ProbAgent);
  %(next_p(T1, Ev, (T2, Prob3)),
  %Prob2 is Prob1 * Prob3)).
next_p(T1, Ev, Ev1, (T2, Prob2)) :-
  next_p((T1, 1), Ev, Ev1, (T2, Prob2)).

may_be_none(_, gap(none)) :- !.
may_be_none(_, gap(msg(S, R, _C))) :- var(S), var(R), !.
may_be_none(T, gap(msg(S, R, C))) :-
  not(next_p(T, gap(msg(S, R, C)), _)), !.

% multi_next((T, Prob), [], Prob) :-
%   !, may_halt(T).
multi_next((_, Prob), [], Prob) :- !.
multi_next((T, ProbT), [Ev|Evs], Prob) :-
  !,
  %next((T, ProbT), Ev, _, (T1, Prob1)),
  next_p(T, Ev, (T1, Prob2)), Prob1 is ProbT * Prob2,
  multi_next((T1, Prob1), Evs, Prob).
multi_next(T, Evs, Prob) :-
  multi_next((T, 1), Evs, Prob).

partial_belongs((T, ProbT, Evs), Ev, TPs, TPsLost) :-
  !,
  findall(
    (T1, ProbT1, Evs1),
    (next_p((T, ProbT), Ev, Ev1, (T1, ProbT1)), append(Evs, [Ev1], Evs1)),
    TPs),
  ((TPs == [])->(TPsLost = [(T, ProbT, Evs)]);(TPsLost = [])).
partial_belongs([], _, [], []) :- !.
partial_belongs([TP|TPs], Ev, TPs3, TPsLost) :-
  !, partial_belongs(TP, Ev, TPs1, TPsLost1),
  partial_belongs(TPs, Ev, TPs2, TPsLost2),
  append(TPs1, TPs2, TPs3),
  append(TPsLost1, TPsLost2, TPsLost).
partial_belongs(T, Ev, TPs, TPsLost) :-
  partial_belongs((T, 1, []), Ev, TPs, TPsLost).

belongs(T, Evs, Prob, N) :-
  findall(P, multi_next(T, Evs, P), Ps),
  length(Ps, N),
  sum_list(Ps, Prob).

% belongs(T, Evs, Prob) :-
%   findall(P, multi_next(T, Evs, P), Ps),
%   sum_list(Ps, Prob).


%match(ET, ET).

% match(msg(a, b, m1), m1) :- !.
% match(msg(b, a, m2), m2) :- !.
% match(msg(a, c, m3), m3) :- !.
% match(msg(c, b, m4), m4) :- !.
% match(msg(d, e, m5), m5) :- !.
% match(msg(alice, bob, ping), ping) :- !.
% match(msg(bob, alice, pong), pong) :- !.
%
% match(ET, (ET, _)) :- !.
% match(ET, ET).


number_of_agents(5).

probAgent(ProbAgent) :-
  number_of_agents(NAgents),
  ProbAgent is 1 / NAgents.


simulate_distributed_agents(TPs, [], TPs) :- !.
simulate_distributed_agents(TPs, [Ev|Evs], TPRes) :- !,
  simulate_distributed_agents(TPs, Ev, TPs1),
  simulate_distributed_agents(TPs1, Evs, TPRes).

simulate_distributed_agents(TPs, Ev, TPsRes) :-
  simulate_distributed_agents_aux(TPs, Ev, TPs1, TPsLost),
  update_distributed_protocols(TPs1, TPsLost, TPsRes).

%simulate_distributed_agents_no_cut(TPs, [], TPs) :- !.
%simulate_distributed_agents_no_cut(TPs, [Ev|Evs], TPRes) :- !,
%  simulate_distributed_agents_no_cut(TPs, Ev, TPs1),
%  simulate_distributed_agents_no_cut(TPs1, Evs, TPRes).

% simulate_distributed_agents_no_cut([], _, []).
% simulate_distributed_agents_no_cut([(ID, TPs)|Others], Ev, [(ID, TPs1)|Others1]) :-
%   involved(ID, Agents1),
%   involved(Ev, Agents2),
%   intersection(Agents1, Agents2, Agents),
%   ((Agents \= []) ->
%     (
%     partial_belongs(TPs, Ev, TPs1, _)
%     );
%     (
%     TPs1 = TPs
%     )),
%   copy_term(Ev, Ev1),
%   simulate_distributed_agents_no_cut(Others, Ev1, Others1).


simulate_distributed_agents_parallel(TPs, [], TPs) :- !.
simulate_distributed_agents_parallel(TPs, [Ev|Evs], TPRes) :- !,
  simulate_distributed_agents_parallel(TPs, Ev, TPs1),
  simulate_distributed_agents_parallel(TPs1, Evs, TPRes).

simulate_distributed_agents_parallel(TPs, Ev, TPsRes) :-
  findall(ThreadID,
    (
      member(TP, TPs),
      simulate_distributed_agents_aux_parallel(TP, Ev, ThreadID)
    ),
    ThreadIDs),
  findall((ID, TPs2, TPsLost),
    (
      member(ThreadID, ThreadIDs),
      thread_join(ThreadID, exited((ID, TPs2, TPsLost)))
    ), TPss),
  findall((ID, TPs2),member((ID, TPs2, _), TPss), TPs1),
  findall((ID, TPs2),member((ID, _, TPs2), TPss), TPsLost),
  update_distributed_protocols(TPs1, TPsLost, TPsRes).

update_distributed_protocols(TPs, [], TPs) :- !.
update_distributed_protocols(TPs, TPsLost, TPs) :-
  findall(P, (member((_, P), TPsLost), P \= []), []), !.
update_distributed_protocols(TPs, TPsLost, TPsRes) :-
  findall((ID1, Gap), (member((ID1,Paths), TPs), sure_gaps(Paths, Gaps1), member(Gap, Gaps1)), Gaps),
  findall((ID1, Gap), (member((ID1,Paths), TPsLost), member((ID1,PathsTPs), TPs), extract_gaps(PathsTPs, Paths, Gaps1), member(Gap, Gaps1)), LostGaps),
  append(Gaps, LostGaps, AllGaps),
  writeln(('AllGaps':AllGaps)),
  findall((ID2, TPs1, LostPaths), (member((ID2, Paths), TPs), update_paths(ID2, Paths, AllGaps, TPs1, LostPaths)), TPs2),
  findall((ID2, TPs1), member((ID2, TPs1, _), TPs2), TPs3),
  findall((ID2, LostPaths), member((ID2, _, LostPaths), TPs2), TPsLost1),
  update_distributed_protocols(TPs3, TPsLost1, TPsRes).

simulate_distributed_agents_aux([], _, [], []).
simulate_distributed_agents_aux([(ID, TPs)|Others], Ev, [(ID, TPs1)|Others1], TPsLost2) :-
  involved(ID, Agents1),
  involved(Ev, Agents2),
  intersection(Agents1, Agents2, Agents),
  ((Agents \= []) ->
    (
    partial_belongs(TPs, Ev, TPs1, TPsLost)
    );
    (
    TPs1 = TPs
    )),
  copy_term(Ev, Ev1),
  simulate_distributed_agents_aux(Others, Ev1, Others1, TPsLost1),
  (TPsLost = [] -> (TPsLost2 = TPsLost1);(TPsLost2 = [(ID, TPsLost)|TPsLost1])).

simulate_distributed_agents_aux_parallel((ID, TPs), Ev, ThreadID) :-
  thread_create(
  (
    involved(ID, Agents1),
    involved(Ev, Agents2),
    intersection(Agents1, Agents2, Agents),
    ((Agents \= []) ->
      (
      partial_belongs(TPs, Ev, TPs1, TPsLost)%, TPs1 \= []
      );
      (
      TPs1 = TPs
      )),
    thread_exit((ID, TPs1, TPsLost))
  ), ThreadID, []).

involved(ta, [a]).
involved(tb, [b]).
involved(tc, [c]).
involved(td, [d]).
involved(te, [e]).
involved(gap(_), [a,b,c,d,e,f,g,h,i,j]) :- !.
involved(msg(Sender, Receiver, _), Set) :-
  ((var(Sender))->
    ((var(Receiver))->
      (Set = []);(Set = [Receiver]));
    ((var(Receiver))->
      (Set = [Sender]);(Set = [Sender, Receiver]))).
involved(tab, [a,b]).
involved(tcd, [c,d]).
involved(tef, [e,f]).
involved(tgh, [g,h]).
involved(tij, [i,j]).

gap_list(0, []) :- !.
gap_list(N, [gap(_)|L]) :-
  N1 is N - 1,
  gap_list(N1, L).

test(Ns) :-
   mas1(T),
   gap_list(10, Gaps),
   project(T, [a,b], Tab),
   project(T, [c,d], Tcd),
   project(T, [e,f], Tef),
   project(T, [g,h], Tgh),
   project(T, [i,j], Tij),
   TPs = [(tab, Tab), (tcd, Tcd), (tef, Tef), (tgh, Tgh), (tij, Tij)],
   simulate_distributed_agents(TPs, Gaps, TPs1),
   findall(N, (member((_,L), TPs1), length(L,N)), Ns).

mas(T) :-
  T = ((m1, 0.2):(m2, 0.5):T) \/
      ((m3, 0.6):(m2, 0.3):epsilon) \/
      ((m4, 0.2):(m2, 0.2):(((m4, 0.8):epsilon) \/ ((m5, 0.2):epsilon))).

mas1(T) :-
  T = (
        ((msg(a, b, m1), 0.1):(msg(b, a, m2), 1):epsilon)|
        ((msg(c, d, m3), 0.6):(msg(d, c, m4), 1):epsilon)|
        ((msg(e, f, m5), 0.1):(msg(f, e, m6), 1):epsilon)|
        ((msg(g, h, m6), 0.1):(msg(h, g, m7), 1):epsilon)|
        ((msg(i, j, m8), 0.1):(msg(j, i, m9), 1):epsilon)
      ).

mas2(T) :-
  T = (
  (
    ((msg(a, b, m1), 0.1):epsilon) |
    ((msg(c, d, m2), 0.1):epsilon) |
    ((msg(e, f, m3), 0.1):epsilon) |
    ((msg(g, h, m4), 0.1):epsilon) |
    ((msg(i, j, m5), 0.1):epsilon) |
    ((msg(k, l, m6), 0.1):epsilon) |
    ((msg(m, n, m7), 0.1):epsilon) |
    ((msg(o, p, m8), 0.1):epsilon) |
    ((msg(q, r, m9), 0.1):epsilon) |
    ((msg(s, t, m10), 0.1):epsilon) |
    ((msg(u, v, m11), 0.1):epsilon) |
    ((msg(w, x, m12), 0.1):epsilon) |
    ((msg(y, z, m13), 0.1):epsilon)
  ) * T).

mas2inc(T, 1) :-
  T =
  ((
  ((msg(a, b, m1), 0.1):epsilon)
  ) * T).
mas2inc(T, 2) :-
  T =
  ((
  ((msg(a, b, m1), 0.1):epsilon) |
  ((msg(c, d, m2), 0.1):epsilon)
  ) * T).
mas2inc(T, 3) :-
  T =
  ((
  ((msg(a, b, m1), 0.1):epsilon) |
  ((msg(c, d, m2), 0.1):epsilon) |
  ((msg(e, f, m3), 0.1):epsilon)
  ) * T).
mas2inc(T, 4) :-
  T =
  ((
  ((msg(a, b, m1), 0.1):epsilon) |
  ((msg(c, d, m2), 0.1):epsilon) |
  ((msg(e, f, m3), 0.1):epsilon) |
  ((msg(g, h, m4), 0.1):epsilon)
  ) * T).

mas2inc(T, 5) :-
  T =
  ((
  ((msg(a, b, m1), 0.1):epsilon) |
  ((msg(c, d, m2), 0.1):epsilon) |
  ((msg(e, f, m3), 0.1):epsilon) |
  ((msg(g, h, m4), 0.1):epsilon) |
  ((msg(i, j, m5), 0.1):epsilon)
  ) * T).

mas2inc(T, 6) :-
  T =
  ((
  ((msg(a, b, m1), 0.1):epsilon) |
  ((msg(c, d, m2), 0.1):epsilon) |
  ((msg(e, f, m3), 0.1):epsilon) |
  ((msg(g, h, m4), 0.1):epsilon) |
  ((msg(i, j, m5), 0.1):epsilon) |
  ((msg(k, l, m6), 0.1):epsilon)
  ) * T).

mas2inc(T, 7) :-
  T =
  ((
  ((msg(a, b, m1), 0.1):epsilon) |
  ((msg(c, d, m2), 0.1):epsilon) |
  ((msg(e, f, m3), 0.1):epsilon) |
  ((msg(g, h, m4), 0.1):epsilon) |
  ((msg(i, j, m5), 0.1):epsilon) |
  ((msg(k, l, m6), 0.1):epsilon) |
  ((msg(m, n, m7), 0.1):epsilon)
  ) * T).

mas2inc(T, 8) :-
  T =
  ((
  ((msg(a, b, m1), 0.1):epsilon) |
  ((msg(c, d, m2), 0.1):epsilon) |
  ((msg(e, f, m3), 0.1):epsilon) |
  ((msg(g, h, m4), 0.1):epsilon) |
  ((msg(i, j, m5), 0.1):epsilon) |
  ((msg(k, l, m6), 0.1):epsilon) |
  ((msg(m, n, m7), 0.1):epsilon) |
  ((msg(o, p, m8), 0.1):epsilon)
  ) * T).

mas2inc(T, 9) :-
  T =
  ((
  ((msg(a, b, m1), 0.1):epsilon) |
  ((msg(c, d, m2), 0.1):epsilon) |
  ((msg(e, f, m3), 0.1):epsilon) |
  ((msg(g, h, m4), 0.1):epsilon) |
  ((msg(i, j, m5), 0.1):epsilon) |
  ((msg(k, l, m6), 0.1):epsilon) |
  ((msg(m, n, m7), 0.1):epsilon) |
  ((msg(o, p, m8), 0.1):epsilon) |
  ((msg(q, r, m9), 0.1):epsilon)
  ) * T).

mas2inc(T, 10) :-
  T =
  ((
  ((msg(a, b, m1), 0.1):epsilon) |
  ((msg(c, d, m2), 0.1):epsilon) |
  ((msg(e, f, m3), 0.1):epsilon) |
  ((msg(g, h, m4), 0.1):epsilon) |
  ((msg(i, j, m5), 0.1):epsilon) |
  ((msg(k, l, m6), 0.1):epsilon) |
  ((msg(m, n, m7), 0.1):epsilon) |
  ((msg(o, p, m8), 0.1):epsilon) |
  ((msg(q, r, m9), 0.1):epsilon) |
  ((msg(s, t, m10), 0.1):epsilon)
  ) * T).

mas2inc(T, 11) :-
  T =
  ((
  ((msg(a, b, m1), 0.1):epsilon) |
  ((msg(c, d, m2), 0.1):epsilon) |
  ((msg(e, f, m3), 0.1):epsilon) |
  ((msg(g, h, m4), 0.1):epsilon) |
  ((msg(i, j, m5), 0.1):epsilon) |
  ((msg(k, l, m6), 0.1):epsilon) |
  ((msg(m, n, m7), 0.1):epsilon) |
  ((msg(o, p, m8), 0.1):epsilon) |
  ((msg(q, r, m9), 0.1):epsilon) |
  ((msg(s, t, m10), 0.1):epsilon) |
  ((msg(u, v, m11), 0.1):epsilon)
  ) * T).

mas2inc(T, 12) :-
  T =
  ((
  ((msg(a, b, m1), 0.1):epsilon) |
  ((msg(c, d, m2), 0.1):epsilon) |
  ((msg(e, f, m3), 0.1):epsilon) |
  ((msg(g, h, m4), 0.1):epsilon) |
  ((msg(i, j, m5), 0.1):epsilon) |
  ((msg(k, l, m6), 0.1):epsilon) |
  ((msg(m, n, m7), 0.1):epsilon) |
  ((msg(o, p, m8), 0.1):epsilon) |
  ((msg(q, r, m9), 0.1):epsilon) |
  ((msg(s, t, m10), 0.1):epsilon) |
  ((msg(u, v, m11), 0.1):epsilon) |
  ((msg(w, x, m12), 0.1):epsilon)
  ) * T).

mas2inc(T, 13) :-
  T =
  ((
  ((msg(a, b, m1), 0.1):epsilon) |
  ((msg(c, d, m2), 0.1):epsilon) |
  ((msg(e, f, m3), 0.1):epsilon) |
  ((msg(g, h, m4), 0.1):epsilon) |
  ((msg(i, j, m5), 0.1):epsilon) |
  ((msg(k, l, m6), 0.1):epsilon) |
  ((msg(m, n, m7), 0.1):epsilon) |
  ((msg(o, p, m8), 0.1):epsilon) |
  ((msg(q, r, m9), 0.1):epsilon) |
  ((msg(s, t, m10), 0.1):epsilon) |
  ((msg(u, v, m11), 0.1):epsilon) |
  ((msg(w, x, m12), 0.1):epsilon) |
  ((msg(y, z, m13), 0.1):epsilon)
  ) * T).

mas3(T) :-
  T = ((((msg(a, b, m1), 0.1):epsilon) |
       ((msg(c, d, m2), 0.1):epsilon) |
       ((msg(e, f, m3), 0.1):epsilon)) * T).

mas4(T) :-
  T =
    ((msg(alice, barbara, buy) :
      (msg(barbara, carol, reserve) :
        (msg(carol, dave, is_available) :
          (((msg(dave, emily, send_book) : (msg(emily, barbara, book_available) : epsilon))
           \/ (msg(dave, frank, order_book) : (msg(frank, barbara, available_in_2_days) : epsilon))))))) * T).

generate_mas(NBranches, NAgentsForBranch, NOperators, Partition, T) :-
  generate_partition(NBranches, NAgentsForBranch, Partition),
  generate_mas(Partition, NOperators, T).

generate_mas([], _, epsilon).
generate_mas([Agents|Partition], NOperators, (Branch|T)) :-
  generate_branch(Agents, NOperators, Branch),
  generate_mas(Partition, NOperators, T).


generate_agents(AgentName, N, Agents) :-
  generate_agents(AgentName, 0, N, Agents).
generate_agents(_, Index, Index, []) :- !.
generate_agents(AgentName, Index, N, [Agent|Agents]) :-
  string_concat(AgentName, Index, Agent),
  Index1 is Index + 1,
  generate_agents(AgentName, Index1, N, Agents).

generate_partition(NBranches, NAgentsForBranch, P) :-
  generate_partition(0, NBranches, NAgentsForBranch, P).
generate_partition(Index, Index, _, []) :- !.
generate_partition(Index, NBranches, NAgentsForBranch, [Agents|P]) :-
  string_concat("ag", Index, AgentName),
  string_concat(AgentName, "_", AgentName1),
  generate_agents(AgentName1, NAgentsForBranch, Agents),
  Index1 is Index + 1,
  generate_partition(Index1, NBranches, NAgentsForBranch, P).

generate_branch(Agents, NOperators, Branch) :-
  generate_branch(Agents, 0, NOperators, Branch).
generate_branch(_, Index, Index, epsilon) :- !.
generate_branch(Agents, Index, NOperators, Branch) :-
  length(Agents, NAgents),
  Pick1 is random(NAgents),
  Pick2 is random(NAgents),
  ((Pick1 = Pick2) -> (Pick3 is (Pick2 + 1) mod NAgents);(Pick3 = Pick2)),
  nth0(Pick1, Agents, Sender, _),
  nth0(Pick3, Agents, Receiver, _),
  ContentNumber is random(NOperators) + 1,
  string_concat("m", ContentNumber, Content),
  Msg = msg(Sender, Receiver, Content),
  Index1 is Index + 1,
  generate_branch(Agents, Index1, NOperators, Branch1),
  random(X),
  ((X > 0.5) ->
    (Branch = Msg:Branch1);
    (Branch = ((Msg:epsilon)|Branch1))).



generate_trace(_, 0, _, _, []) :- !.
generate_trace(epsilon, _, _, _, []) :- !.
generate_trace(T, Length, ProbNoise, MessageNoise, [Ev|Trace]) :-
  next(T, msg(Sender, Receiver, Content), T1),
  random(R),
  ((R < ProbNoise) ->
    (
      random(R1),
      ((R1 > MessageNoise) -> (Sender1 = Sender);(true)),
      random(R2),
      ((R2 > MessageNoise) -> (Receiver1 = Receiver);(true)),
      random(R3),
      ((R3 > MessageNoise) -> (Content1 = Content);(true)),
      Ev = gap(msg(Sender1, Receiver1, Content1))
    );
    (Ev = msg(Sender, Receiver, Content))),
  Length1 is Length - 1,
  generate_trace(T1, Length1, ProbNoise, MessageNoise, Trace).

generate_traces(T, Length, ProbNoise, MessageNoise, N, Traces) :-
  retractall(count(_)),
  asserta(count(N)),
  findall(Trace,
    (
      generate_trace(T, Length, ProbNoise, MessageNoise, Trace),
      count(N1),
      retractall(count(_)),
      N2 is N1 - 1,
      asserta(count(N2)),
      ((N1 = 0) ->
        (!, fail);
        (true))
    ),
    Traces
  ).

assert_traces(ID, T, Length, ProbNoise, MessageNoise, N) :-
  generate_traces(T, Length, ProbNoise, MessageNoise, N, Traces),
  retractall(traces(_, _)),
  asserta(traces(ID, Traces)).

% generate_trace_file(Filename, Length, T) :-
%   open(Filename, write, Stream),
%   findall(Trace, (generate_trace(T, Length, Trace), writeln(Stream, Trace)), _),
%   close(Stream).

list_of_lengths(N, L) :-
  N1 is N + 1,
  list_of_lengths(1, N1, L).
list_of_lengths(N, N, []) :- !.
list_of_lengths(I, N, [I|L]) :-
  I >= 0,
  I1 is I + 1,
  list_of_lengths(I1, N, L).

list_of_prob_noises(Min, Max, L) :-
  list_of_prob_noises_aux(Min, Max, L).
list_of_prob_noises_aux(I, Max, []) :-
  I > Max, !.
list_of_prob_noises_aux(I, Max, [I|L]) :-
  I >= 0,
  I1 is I + 0.1,
  list_of_prob_noises_aux(I1, Max, L).

create_output_file(ID, T, MaxLength, NTests, MinProbNoise, MaxProbNoise, MinProbMsgNoise, MaxProbMsgNoise, Partition) :-
  list_of_lengths(MaxLength, L),
  list_of_prob_noises(MinProbNoise, MaxProbNoise, ProbNoises),
  list_of_prob_noises(MinProbMsgNoise, MaxProbMsgNoise, MsgNoises),
  string_concat(ID, '.csv', Filename),
  open(Filename, write, Stream),
  writeln(Stream, 'Length; ProbNoise; MsgNoise; Cen.; Dec.'),
  findall(_,
    (
      member(ProbNoise, ProbNoises),
      ProbNoise > 0.0,
      member(MessageNoise, MsgNoises),
      member(Length, L),
      write('ProbNoise: '), write(ProbNoise),
      write(', MsgNoise: '), write(MessageNoise),
      write(', Length: '), writeln(Length),
      generate_traces(T, Length, ProbNoise, MessageNoise, NTests, Traces),
      probabilize(T, PT),
      test_trace_expression(PT, Traces, AvgTime1),
      writeln('Centralized ended'),
      test_trace_expression(T, Traces, Partition, false, AvgTime2),
      writeln('Decentralized ended'),
      %test_trace_expression(T, Traces, Partition, true, AvgTime2),
      %writeln('Decentralized ended'),
      %test_trace_expression(T, Traces, Partition, true, AvgTime3),
      %writeln('Decentralized par. ended'),
      write(Stream, Length),
      write(Stream, '; '),
      write(Stream, ProbNoise),
      write(Stream, '; '),
      write(Stream, MessageNoise),
      write(Stream, '; '),
      write(Stream, AvgTime1),
      write(Stream, '; '),
      writeln(Stream, AvgTime2),
      %write(Stream, '; '),
      %writeln(Stream, AvgTime3),
      flush_output(Stream)
    ),
  _),
  close(Stream).

test_trace_expression(T, Traces, AvgTime) :-
  retractall(count(_)),
  asserta(count(0)),
  findall(
    DTime,
    (
      member(Trace, Traces),
      get_time(StartTime),
      belongs(T, Trace, _, _),
      get_time(EndTime),
      DTime is EndTime - StartTime%,
      %writeln(DTime)
    ),
    DTimes),
  length(DTimes, N),
  sum_list(DTimes, Total),
  AvgTime is Total / N.

test_trace_expression(T, Traces, Partition, Parallel, AvgTime) :-
  findall(Agent, (member(S, Partition), member(Agent, S)), Agents),
  retractall(involved(gap(_), _)),
  asserta(involved(gap(_), Agents)),
  findall((ID, TPr),
    (
      member(S, Partition),
      partition_to_string(S, String),
      string_concat("t", String, ID),
      retractall(involved(ID, _)),
      asserta(involved(ID, S)),
      project(T, S, TPr1),
      probabilize(TPr1, TPr)
    ),
    TPs
  ),
  findall(
    DTime,
    (
      member(Trace, Traces),
      get_time(StartTime),
      (Parallel ->
        (simulate_distributed_agents_parallel(TPs, Trace, _));
        %((Cut ->
        (simulate_distributed_agents(TPs, Trace, _))
          %(simulate_distributed_agents_no_cut(TPs, Trace, _))
      ),
      get_time(EndTime),
      DTime is EndTime - StartTime%,
      %writeln(DTime)
    ),
    DTimes),
  length(DTimes, N),
  sum_list(DTimes, Total),
  AvgTime is Total / N.

partition_to_string([], "").
partition_to_string([H|T], String) :-
  partition_to_string(T, String1),
  string_concat(H, String1, String).

distributed_agents(TPs) :-
  Ta = (((m1,0.3):(m2, 0.8):epsilon)\/((m3,0.4):(m2, 0.8):epsilon)),
  Tb = ((m1,0.3):(m4,0.2):epsilon)\/((m2,0.6):epsilon)\/((m4, 0.4):(m2, 0.6):(m4, 0.6):epsilon),
  %Tc = ((m2,0.2):epsilon),
  %Td = ((m3,0.4):(m4,0.8):epsilon),
  %Te = ((m4,0.8):epsilon)
  TPs = [(ta,Ta),(tb,Tb)].%,(tc,Tc)].%,(td,Td),(te,Te)].

test(Events, T1, T2, Prob) :-
  T1 = (ping:pong:T1),
  T2 = (((ping, 0.8):(((pong, 0.4):T2)\/((ping, 0.6):T2)))\/((pong, 0.2):(((ping, 0.7):T2)\/((pong, 0.3):T2)))),
  belongs(T1/\T2, Events, Prob).


update_paths(_, Paths, [], Paths, []).
update_paths(ID, Paths, [(ID1, (_Gap, _Step))|Gaps], Paths2, LostPaths) :-
  ID = ID1, !,
  update_paths(ID, Paths, Gaps, Paths2, LostPaths).
update_paths(ID, Paths, [(ID1, (Gap, Step))|Gaps], Paths2, LostPaths) :-
  update_paths(ID, Paths, ID1, Step, Gap, Paths1, LostPaths1),
  update_paths(ID, Paths1, Gaps, Paths2, LostPaths2),
  append(LostPaths1, LostPaths2, LostPaths).

update_paths(_, [], _, _, _, [], []).
update_paths(ID, [(T, Prob, Evs)|Paths], ID1, Step, gap(Ev), UpdatedPaths, LostPaths) :-
  ((is_still_valid(ID, Evs, 0, Step, ID1, gap(Ev)))->
    (UpdatedPaths = [(T, Prob, Evs)|Paths1], LostPaths = LostPaths1);
    (UpdatedPaths = Paths1), LostPaths = [(T, Prob, Evs)|LostPaths1]),
  update_paths(ID, Paths, ID1, Step, gap(Ev), Paths1, LostPaths1).
update_paths(ID, [(T, Prob, Evs)|Paths], ID1, Step, not(gap(Ev)), UpdatedPaths, LostPaths) :-
  ((is_still_valid(ID, Evs, 0, Step, ID1, not(gap(Ev))))->
    (UpdatedPaths = [(T, Prob, Evs)|Paths1], LostPaths = LostPaths1);
    (UpdatedPaths = Paths1), LostPaths = [(T, Prob, Evs)|LostPaths1]),
  update_paths(ID, Paths, ID1, Step, not(gap(Ev)), Paths1, LostPaths1).

is_still_valid(_, [], _, _, _, _) :- !.
is_still_valid(_ID, [gap(Ev1)|_], Step, Step, ID1, gap(none)) :-
  !, ((Ev1=none, !);(involved(Ev1, S1), involved(ID1, S2), intersection(S1, S2, []))).
is_still_valid(ID, [gap(Ev1)|_], Step, Step, ID1, not(gap(none))) :-
  !, ((Ev1 = none, involved(ID, S1), involved(ID1, S2), intersection(S1, S2, []));
  (involved(Ev1, S1), involved(ID1, S2), intersection(S1, S2, S), S \== [])).
  %!, ((Ev1=none, !);(involved(Ev1, S1), involved(ID1, S2), intersection(S1, S2, []))).
is_still_valid(ID, [gap(Ev1)|_], Step, Step, _ID1, gap(Ev)) :-
  !, (Ev1 = Ev; (Ev1 = none, involved(ID, S1), involved(Ev, S2), intersection(S1, S2, []))).
is_still_valid(_ID, [gap(Ev1)|_], Step, Step, _ID1, not(gap(Ev))) :-
  !, Ev1 \= Ev.
  %(Ev1 = none, involved(ID, S1), involved(Ev, S2), intersection(S1, S2, []))).
is_still_valid(ID, [gap(_)|Evs], Index, Step, ID1, Gap) :-
  !, Index1 is Index + 1,
  is_still_valid(ID, Evs, Index1, Step, ID1, Gap).
is_still_valid(ID, [_|Evs], Index, Step, ID1, Gap) :-
  is_still_valid(ID, Evs, Index, Step, ID1, Gap).

extract_gaps(TPs, TPsLost, Gaps) :-
  findall(Path, member((_, _, Path), TPsLost), Paths),
  extract_gaps_aux(TPs, Paths, Gaps).

extract_gaps_aux(TPs, Paths, Gaps) :-
  extract_gaps_aux(TPs, Paths, 0, Gaps).
extract_gaps_aux(_, [], _, []) :- !.
extract_gaps_aux(TPs, Paths, Step, Gaps2) :-
  (get_heads(Paths, Heads, Tails), !,
  ((member(Ev, Heads), Ev \= gap(_)) -> %Ev \= none
    (Gaps = [], Step1 = Step);
    (findall((not(gap(Ev)), Step), (member(gap(Ev), Heads), not_in_paths(gap(Ev), Step, TPs)), Gaps), Step1 is Step + 1)
  ),
  extract_gaps_aux(TPs, Tails, Step1, Gaps1),
  append(Gaps, Gaps1, Gaps2));
  (Gaps2 = []).

not_in_paths(gap(Ev), Step, TPs) :- %writeln((gap(Ev), TPs)),
  findall(Path,
    (
      member((_, _, Path), TPs),
      nth0(Step, Path, gap(Ev))
    ),
  []).

sure_gaps(TPs, Gaps) :-
  findall(Path, member((_, _, Path), TPs), Paths),
  sure_gaps_aux(Paths, Gaps).

sure_gaps_aux(Paths, Gaps) :-
  sure_gaps_aux(Paths, 0, Gaps).
sure_gaps_aux([], _, []) :- !.
sure_gaps_aux(Paths, Step, Gaps) :-
  (get_heads(Paths, Heads, Tails), !,
  ((maplist(=(gap(Ev)), Heads)) -> %Ev \= none
    (Gaps = [(gap(Ev), Step)|Gaps1]);
    (Gaps = Gaps1)),
  (member(gap(_), Heads) -> (Step1 is Step + 1);(Step1 = Step)),
  sure_gaps_aux(Tails, Step1, Gaps1));
  (Gaps = []).

get_heads([], [], []).
get_heads([[Head|Tail]|Lists], [Head|Heads], [Tail|Tails]) :-
  get_heads(Lists, Heads, Tails).


/* ************************************************************************** */
/* 							DYNAMIC PROJECTION				 			      */
/* ************************************************************************** */

project(T, ProjectedAgents, ProjectedType) :-
	empty_assoc(A),
	project(A, 0, -1, T, ProjectedAgents, ProjectedType).

project(_Assoc, _Depth, _DeepestSeq, epsilon, _ProjectedAgents, epsilon):- !.

project(Assoc, _Depth, DeepestSeq, Type, _ProjectedAgents, ProjectedType) :-
get_assoc(Type,Assoc,(AssocProjType,LoopDepth)),!,(LoopDepth =< DeepestSeq -> ProjectedType=AssocProjType; ProjectedType=epsilon).
/*
project(Assoc, Depth, DeepestSeq, (IntType:Type1), ProjectedAgents, ProjectedType) :-
IntType \= (_, _),
!,
put_assoc((IntType:Type1),Assoc,(ProjectedType,Depth),NewAssoc),
(involves(IntType, ProjectedAgents) ->
    IncDepth is Depth+1,project(NewAssoc,IncDepth,Depth,Type1,ProjectedAgents,ProjectedType1),ProjectedType=(IntType:ProjectedType1);
    project(NewAssoc,Depth,DeepestSeq,Type1,ProjectedAgents,ProjectedType)).
*/

project(Assoc, Depth, DeepestSeq, ((IntType, Prob):Type1), ProjectedAgents, ProjectedType) :-
!,
put_assoc(((IntType, Prob):Type1),Assoc,(ProjectedType,Depth),NewAssoc),
(involves(IntType, ProjectedAgents) ->
    IncDepth is Depth+1, project(NewAssoc,IncDepth,Depth,Type1,ProjectedAgents,ProjectedType1),ProjectedType=((IntType, Prob):ProjectedType1);
    project(NewAssoc,Depth,DeepestSeq,Type1,ProjectedAgents,ProjectedType)).

project(Assoc, Depth, DeepestSeq, (IntType:Type1), ProjectedAgents, ProjectedType) :-
!,
put_assoc((IntType:Type1),Assoc,(ProjectedType,Depth),NewAssoc),
(involves(IntType, ProjectedAgents) ->
    IncDepth is Depth+1, project(NewAssoc,IncDepth,Depth,Type1,ProjectedAgents,ProjectedType1),ProjectedType=(IntType:ProjectedType1);
    project(NewAssoc,Depth,DeepestSeq,Type1,ProjectedAgents,ProjectedType)).

project(Assoc, Depth, DeepestSeq, (Type1|Type2), ProjectedAgents, ProjectedType) :-
!,
put_assoc((Type1|Type2),Assoc,(ProjectedType,Depth),NewAssoc),
IncDepth is Depth+1,
project(NewAssoc, IncDepth, DeepestSeq, Type1, ProjectedAgents, ProjectedType1),
project(NewAssoc, IncDepth, DeepestSeq, Type2, ProjectedAgents, ProjectedType2),
ProjectedType=(ProjectedType1|ProjectedType2).

project(Assoc, Depth, DeepestSeq, (Type1\/Type2), ProjectedAgents, ProjectedType) :-
!,
put_assoc((Type1\/Type2),Assoc,(ProjectedType,Depth),NewAssoc),
IncDepth is Depth+1,
project(NewAssoc, IncDepth, DeepestSeq, Type1, ProjectedAgents, ProjectedType1),
project(NewAssoc, IncDepth, DeepestSeq, Type2, ProjectedAgents, ProjectedType2),
ProjectedType=(ProjectedType1\/ProjectedType2).

project(Assoc, Depth, DeepestSeq, (Type1/\Type2), ProjectedAgents, ProjectedType) :-
!,
put_assoc((Type1/\Type2),Assoc,(ProjectedType,Depth),NewAssoc),
IncDepth is Depth+1,
project(NewAssoc, IncDepth, DeepestSeq, Type1, ProjectedAgents, ProjectedType1),
project(NewAssoc, IncDepth, DeepestSeq, Type2, ProjectedAgents, ProjectedType2),
ProjectedType=(ProjectedType1/\ProjectedType2).

project(Assoc, Depth, DeepestSeq, (Type1*Type2), ProjectedAgents, ProjectedType) :-
!,
put_assoc((Type1*Type2),Assoc,(ProjectedType,Depth),NewAssoc),
IncDepth is Depth+1,
project(NewAssoc, IncDepth, DeepestSeq, Type1, ProjectedAgents, ProjectedType1),
((may_halt(Type1))->
  (project(NewAssoc, IncDepth, DeepestSeq, Type2, ProjectedAgents, ProjectedType2));
  (project(NewAssoc, IncDepth, Depth, Type2, ProjectedAgents, ProjectedType2))),
ProjectedType=(ProjectedType1*ProjectedType2).

project(Assoc, Depth, DeepestSeq, (IntType>>Type1), ProjectedAgents, ProjectedType) :-
!,
put_assoc((IntType>>Type1),Assoc,(ProjectedType,Depth),NewAssoc),
%(involves(IntType, ProjectedAgents) ->
IncDepth is Depth+1,
project(NewAssoc,IncDepth,DeepestSeq,Type1,ProjectedAgents,ProjectedType1),ProjectedType=(IntType>>ProjectedType1).%;

project(Assoc, Depth, DeepestSeq, var(X,Type1), ProjectedAgents, ProjectedType) :-
!,
put_assoc(var(X,Type1),Assoc,(ProjectedType,Depth),NewAssoc),
IncDepth is Depth + 1,
project(NewAssoc, IncDepth, DeepestSeq, Type1, ProjectedAgents, ProjectedType1),
ProjectedType = var(X, ProjectedType1).

/****************************************************************************/
/* 	  INVOLVES PREDICATE: can be redefined to consider agent roles      */
/****************************************************************************/

involves(IntType, List) :-
  match(Message, IntType),
  Message =.. [msg, Sender, Receiver | _T],
  (member(Sender, List);
  member(Receiver, List)).

% involves(IntType, List) :-
%   match(Action, IntType),
%   Action =.. [act, agent(A), content(_C) | _T],
%   member(A, List).

/*******************************************************************************************/
/*                              PARAMETRIC TRACE EXPRESSIONS                               */
/*******************************************************************************************/

/* Transition rules */

% next transition function (parametric version)
next( ET:T, E, T, S) :-
  genvar(ET, ETFree, S1),
  match( E, ETFree),
  clear(S1, S).
next( T1\/_, E, T2, S) :-
  next( T1, E, T2, S).
next( _\/T1, E, T2, S) :-
  !, next( T1, E, T2, S).
next( T1|T2, E, T, S) :-
  next( T1, E, T3, S),
  fork(T3, T2, T).
next( T1|T2, E, T, S) :-
  !, next( T2, E, T3, S),
  fork(T1, T3, T).
next( T1*T2, E, T, S) :-
  next( T1, E, T3, S),
  concat(T3, T2, T).
next( T1*T2, E, T3, S) :-
  !, may_halt(T1),
  next( T2, E, T3, S).
next( T1/\T2, E, T, S) :-
  !, next( T1, E, T3, S1),
  next( T2, E, T4, S2),
  merge(S1, S2, S),
  conj(T3, T4, T).
next( ET>>T, E, ET>>T1, S) :-
  event( E),
  genvar(ET, ETFree, S1),
  (match( E, ETFree) *->
    (clear(S1, S2),
     next( T, E, T1, S3),
     merge(S2, S3, S));
     (T=T1, S = [])).
next( var(X, T), E, T2, S) :-
  next( T, E, T1, S1),
  (syntactic_member_couples((X=V), S1) ->
    (substitution(T1, (X=V), T2),% !,
     remove((X=V), S1, S));
    (T2 = var(X, T1), S = S1)).
% (main)
next( T, E, T1) :-
  next( T, E, T1, S), S = [].

does_not_halt(_:_).
does_not_halt(T1\/T2) :- !, does_not_halt(T1), does_not_halt(T2).
does_not_halt(T1|T2) :- (does_not_halt(T1), !; does_not_halt(T2)).
does_not_halt(T1*T2) :- (does_not_halt(T1), !; does_not_halt(T2)).
does_not_halt(T1/\T2) :- (does_not_halt(T1), !; does_not_halt(T2)).
does_not_halt(_>>T) :- !, does_not_halt(T).
does_not_halt(var(_, T)) :- !, does_not_halt(T).

% may_halt predicate iterated on the trace expression
% Example: T = msg1:epsilon, may_eventually_halt(T).
% Example: T = msg1:T, not(may_eventually_halt(T)).
may_eventually_halt( T) :-
  empty_assoc(A),
  may_eventually_halt( T, A).
may_eventually_halt(_, T, A) :-
  get_assoc(T, A, _), !, fail.
may_eventually_halt(_, T, _) :- may_halt(T), !.
may_eventually_halt( T, A) :-
  put_assoc(T, A, _, A1),
  next( T, _, T1), may_eventually_halt( T1, A1).


% Predicate which takes an event type and a variable ('$VAR'(N)) returning
% a new event type where all '$VAR'(N) are substituted with free variables
% and a list of substitutions recording the association between each ground
% variables with the new free variables.
% Example: genvar(ping('$VAR'(0)), ping(X), ['$VAR'(0)=X], '$VAR'(0))
% Example: genvar(ping('$VAR'(1)), ping('$VAR'(1)), [], '$VAR'(0))
genvar(ET, ETFree, [(X=V)], X) :-
  genvar(ET, ETFree, [(X=V)]), !.
genvar(ET, ET, [], _).

% Predicate which takes an event type returning a new event type where
% all '$VAR'(_) are substituted with free variables and a list of substitutions recording the association between each ground
% variables with the new free variables.
% Example: genvar(ping('$VAR'(0)), ping(X), ['$VAR'(0)=X])
% Example: genvar(ping('$VAR'(1)), ping(X), ['$VAR'(1)=X])
% Example: genvar(pong('$VAR'(0), '$VAR'(1)), pong(X, Y), ['$VAR'(0)=X, '$VAR'(1)=Y])
genvar(ET, ETFree, S) :-
  functor(ET, F, N),
  (N == 0 -> (ETFree = ET, S = []);
  (findall(Arg, arg(_, ET, Arg), Args),
  functor(ETFree, F, N),
  genvar_aux(Args, ETFree, 1, S))).
genvar_aux([], _, _, []).
genvar_aux([H1|T1], ETFree, N, [(H1=Arg)|S]) :-
  functor(H1, '$VAR', _), !,
  arg(N, ETFree, Arg),
  N1 is N+1,
  genvar_aux(T1, ETFree, N1, S).
genvar_aux([H1|T1], ETFree, N, S) :-
  arg(N, ETFree, Arg),
  H1 = Arg,
  N1 is N+1,
  genvar_aux(T1, ETFree, N1, S).

% Substitution function -
% It is used to substitute a variable inside an event types
% Example: substitution(ping('$VAR'(0)):epsilon, ('$VAR'(0)=a), ping(a):epsilon)
substitution(T, S, T1) :-
  substitution_aux(T, S, T1), !.
substitution_aux(epsilon, _, epsilon).
substitution_aux(ET:T, (X=V), ET1:T1) :-
  genvar(ET, ET1, S, X),
  ((member((X=V), S), !);true),
  substitution_aux(T, (X=V), T1).
substitution_aux(T1\/T2, S, T3\/T4) :-
  substitution_aux(T1, S, T3),
  substitution_aux(T2, S, T4).
substitution_aux(T1|T2, S, T3|T4) :-
  substitution_aux(T1, S, T3),
  substitution_aux(T2, S, T4).
substitution_aux(T1*T2, S, T3*T4) :-
  substitution_aux(T1, S, T3),
  substitution_aux(T2, S, T4).
substitution_aux(T1/\T2, S, T3/\T4) :-
  substitution_aux(T1, S, T3),
  substitution_aux(T2, S, T4).
substitution_aux(ET>>T, (X=V), ET1>>T1) :-
  genvar(ET, ET1, S, X),
  ((member((X=V), S), !);true),
  substitution_aux(T, (X=V), T1).
substitution_aux(var(X, T), (X=_), var(X, T)) :- !.
substitution_aux(var(X, T), S, var(X, T1)) :-
  substitution_aux(T, S, T1).

% Member function with syntactic equality (on couples)
% Example: syntactic_member_couples((X=a), [(Y=a), (Z=b), (X=a)]) -> true
% Example: syntactic_member_couples((X=a), [(Y=a), (Z=b)]) -> false
syntactic_member_couples((E=V), [(H=V)|_]) :-
  E == H, !.
syntactic_member_couples(E, [_|T]) :-
  syntactic_member_couples(E, T).

% Member function with syntactic equality (on singleton)
syntactic_member(E, [H|_]) :-
  E == H, !.
syntactic_member(E, [_|T]) :-
  syntactic_member(E, T).

% Add a new substitution to the substitution set
% Example: add((X=a), [X=a], [X=a]).
% Example: add((X=a), [], [X=a]).
% Example: add((X=a), [X=b], _) -> false
add(E, [], [E]).
add((E=X), [(H=V)|T], [(H=V)|T1]) :-
  E \== H, !,
  add((E=X), T, T1).
add((_=V), [(H=V)|T], [(H=V)|T]).

% Remove a substitution from the substitution set
% Example: remove((X=a), [X=a], []).
% Example: remove((X=a), [], []).
% Example: remove((X=a), [X=b], _) -> false
remove(_, [], []).
remove((E=X), [(H=V)|T], [(H=V)|T1]) :-
  E \== H, !,
  remove((E=X), T, T1).
remove((E=V), [(_=V)|T], T1) :-
  remove((E=V), T, T1).

% Merge two substitution sets
% Example: merge([(X=a)], [(Y=b)], [Y=b, X=a])
% Example: merge([(X=a)], [(X=a)], [X=a])
% Example: merge([(X=a)], [(X=b)], _) -> false
merge([], L, L).
merge([H|T], L, L2) :-
  add(H, L, L1),
  merge(T, L1, L2), !.

% Remove from a substitution set all couples (_=_)
% Example: clear([(X=a), (Y=b)], [(X=a), (Y=b)])
% Example: clear([(X=a), (Y=_)], [(X=a)])
clear([], []).
clear([(_=Y)|T], T1) :-
  var(Y), !, clear(T, T1).
clear([H|T], [H|T1]) :-
  clear(T, T1).

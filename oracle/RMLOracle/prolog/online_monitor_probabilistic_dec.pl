:- use_module(library(http/websocket)).
:- use_module(library(http/thread_httpd)).
:- use_module(library(http/http_dispatch)).
:- use_module(library(http/http_client)).
:- use_module(library(http/json)).
:- use_module(library(http/json_convert)).
:- use_module(library(http/http_json)).

:- use_module(trace_expressions_semantics_probabilistic).

:- http_handler(/,http_upgrade_to_websocket(manage_event, []),[]). %%% default options for both the websocket and the http handler

%% arguments
%% the server expects a required first argument: the filename containing the specified trace expression
%% second optional argument: a log file, if not provided no logging is performed

%% example:
%% swipl -p node=prolog prolog/server.pl -- http/omitted\ body/204\ response/spec.pl prolog_server_log.txt
%% -p node=prolog
%%          required option to indicate the path to func_match.pl (event domain implementation)
%%  http/omitted\ body/204\ response/spec.pl
%%          the trace expression (required argument)
%%  prolog_server_log.txt
%%          logging enabled to file prolog_server_log.txt (optional argument)

% load specification

:- current_prolog_flag(argv, [Spec|L]), L = [Port|L1], L1 = [Port1|L2], L2 = [Port2|_], asserta(port(Port)), asserta(port1(Port1)), asserta(port2(Port2)), use_module(Spec).

:- dynamic current_state/1.

server(Port) :- http_server(http_dispatch,[port('127.0.0.1':Port),workers(1)]). %% one worker to guarantee event sequentiality
%server(Port) :- http_server(http_dispatch,[port('192.168.1.250':Port),workers(1)]). %% one worker to guarantee event sequentiality

log(Log) :-
    nb_getval(log,Stream), Stream\==null->  %% optional logging of server activity
	(Log=(TE,E)->
	     writeln(Stream,"Trace expression:"),writeln(Stream,TE),writeln(Stream,"Event: "),writeln(Stream,E);
	 Log=(TE,E,error), writeln(Stream,"Trace expression:"),writeln(Stream,TE),writeln(Stream,"Event: "),writeln(Stream,E),writeln(Stream,"Error")),
	nl(Stream),
	flush_output(Stream);
    true.

manage_event(WebSocket) :-
    ws_receive(WebSocket, Msg, [format(json),value_string_as(string)]), %% value_string_as(atom) passed as option to json_read_dict/3
    get_time(Start),
    %writeln('Monitor has observed: '),
    %writeln(Msg),
    port(Port),
    (Msg.opcode==close ->
	     true;
	       E2=Msg.data,
         %writeln('Message data: '),
         %writeln(E2),
         %nb_getval(state,TPs),
         (recorded(state, current_state(TPs)) -> true; (trace_expression(_, TE), TPs = [(TE, 1, [])], recorda(state, current_state(TPs)))),
         %writeln(TPs),
         (string(E2) -> term_string(E, E2); E = E2),
         (is_list(E) ->
         (update_paths(Port, TPs, E, TPs1, _LostPaths), recorda(state, current_state(TPs1)));
         ((match(E, gap) -> E1=gap(_); E=E1),
         % for experiments
         %random(R),
         %((R < 0.7) -> (next(TE1,E,TE2), nb_setval(state,TE2), Reply=E);(term_string(TE1, TE1Str), Reply=(_{}.put(E).put(_{error:true, spec:TE1Str})))),
         %(recorded(0, T) -> writeln('test:'), writeln(T); true),
         ((
         partial_belongs(TPs,E1,TPs1,TPsLost), TPs1\=[])->
          send_gap(Port, TPs1, TPsLost),
           recorda(state, current_state(TPs1)),Reply=E;
           Reply=(_{}.put(E).put(_{error:true, spec:"prob"}))),
	       atom_json_dict(Json,Reply,[as(string)]),
         get_time(End),
         DTime is End - Start,
         writeln(DTime),
	       ws_send(WebSocket, string(Json)))),
	       manage_event(WebSocket)).

send_gap(ID, TPs, TPsLost) :-
  findall((ID, SureGap), (sure_gaps(TPs, SureGaps1), member(SureGap, SureGaps1)), SureGaps),
  findall((ID, LostGap), (extract_gaps(TPs, TPsLost, LostGaps1), member(LostGap, LostGaps1)), LostGaps),
  append(SureGaps, LostGaps, Gaps),
  nb_getval(ws, WS),
  (Gaps = [] -> SGaps = []; term_string(Gaps, SGaps)),
  atom_json_dict(Json, SGaps, [as(string)]),
  ws_send(WS, string(Json)).

exception(undefined_global_variable, state, retry) :- trace_expression(_, TE), nb_setval(state,[(TE, 1, [])]).
exception(undefined_global_variable, log, retry) :- (current_prolog_flag(argv, [_,LogFile|_])->open(LogFile,append,Stream);Stream=null),nb_setval(log, Stream).
exception(undefined_global_variable, ws, retry) :-
  port2(Port),
  string_concat('ws://127.0.0.1:', Port, URL),
  http_open_websocket(URL, WS, []),
  nb_setval(ws, WS).

:- port(Port), port1(Port1), writeln(Port), atom_number(Port, P), atom_number(Port1, P1), server(P), server(P1).

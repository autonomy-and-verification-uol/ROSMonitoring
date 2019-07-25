% MIT License
%
% Copyright (c) [2019] [Davide Ancona, Luca Franceschini, Angelo Ferrando, Viviana Mascardi]
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.


:- use_module(library(http/websocket)).
:- use_module(library(http/thread_httpd)).
:- use_module(library(http/http_dispatch)).
:- use_module(library(http/http_client)).
:- use_module(library(http/json)).
:- use_module(library(http/json_convert)).
:- use_module(library(http/http_json)).

:- use_module(trace_expressions_semantics).

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

:- current_prolog_flag(argv, [Spec|_]), use_module(Spec).

server(Port) :- http_server(http_dispatch,[port('127.0.0.1':Port),workers(1)]). %% one worker to guarantee event sequentiality
%server(Port) :- http_server(http_dispatch,[port('192.168.1.250':Port),workers(1)]). %% one worker to guarantee event sequentiality

log(Log) :-
    nb_getval(log,Stream), Stream\==null->  %% optional logging of server activity
	(Log=(TE,E)->
	     writeln(Stream,"Trace expression:"),writeln(Stream,TE),writeln(Stream,"Event: "),writeln(Stream,E);
	 writeln(Stream,"Error")),
	nl(Stream),
	flush_output(Stream);
    true.

why(Clauses) :- findall((ET, Dict, Clause), clause(match(Ev, ET), (deep_subdict(Dict, Ev), Clause)), Clauses).

manage_event(WebSocket) :-
    ws_receive(WebSocket, Msg, [format(json),value_string_as(string)]), %% value_string_as(atom) passed as option to json_read_dict/3
    writeln('Monitor has observed: '),
    writeln(Msg),
    (Msg.opcode==close ->
	     true;
	       E=Msg.data,
         writeln('Message data: '),
         writeln(E),
	       nb_getval(state,TE1),
	       log((TE1,E)),
         (next(TE1,E,TE2) -> nb_setval(state,TE2),Reply=E; term_string(TE1, TE1Str), Reply=(_{}.put(E).put(_{error:true, spec:TE1Str}))),
	       atom_json_dict(Json,Reply,[as(string)]),
	       ws_send(WebSocket, string(Json)),
	       manage_event(WebSocket)).


exception(undefined_global_variable, state, retry) :- trace_expression(_, TE), nb_setval(state,TE).
exception(undefined_global_variable, log, retry) :- (current_prolog_flag(argv, [_,LogFile|_])->open(LogFile,append,Stream);Stream=null),nb_setval(log, Stream).

:- server(8080).

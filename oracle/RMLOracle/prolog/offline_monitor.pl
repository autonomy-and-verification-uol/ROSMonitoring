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

:- use_module(library(http/json)).
:- use_module(trace_expressions_semantics).
:- initialization(main).

%% arguments
%% - a specification file
%% - a log file containing the trace
%% optional
%% - --silent
%% - --reject

main :-
	current_prolog_flag(argv, [SpecFile, TraceFile | _]), !,
	catch(
		(use_module(SpecFile), trace_expression(_, TraceExp)),
		_,
		(write('File not found\n'), halt(1))),
	catch(
		read_trace(TraceFile, TraceStream),
		_,
		(write('Illegal JSON object\n'), halt(1))),
	(verify(TraceStream, TraceExp, 1) -> Accepted=true ; Accepted=false),
	close(TraceStream),
	(reject -> negate(Accepted, Result) ; Result=Accepted),
	(Result=true ->
		(log('Execution terminated correctly\n'), halt(0)) ;
		(log('Trace did not match specification\n'), halt(1))).

main :-
	write('expected args: <spec file> <trace file>\n'),
	halt(1).

negate(false, true).
negate(true, false).

% true if --silent flag was given
silent :-
	current_prolog_flag(argv, [_, _ | Arguments]),
	member('--silent', Arguments).

% true if --reject flag was given
reject :-
	current_prolog_flag(argv, [_, _ | Arguments]),
	member('--reject', Arguments).

% only print if not in silent mode
log(X) :- silent -> true ; write(X).
lognl  :- silent -> true ; nl.

read_trace(TraceFile, TraceStream) :-
    catch(
    	open(TraceFile, read, TraceStream),
    	_,
    	(write('trace file not found'), nl, halt(1))).

verify(TraceStream, TraceExp, EventId) :-
	at_end_of_stream(TraceStream) ->
		verify_end(TraceExp) ;
		verify_events(TraceStream, TraceExp, EventId).

% check wether end of trace is allowed
verify_end(TraceExp) :- may_halt(TraceExp) ->
	true ;
	(log('Unexpected end of trace\n'), false).

% verify one event and then proceed recursively
verify_events(TraceStream, TraceExp, EventId) :-
	json_read_dict(TraceStream, Event, [end_of_file(empty{})]),
	writeln(TraceExp), writeln(Event),
	(Event=empty{} -> (verify_end(TraceExp));(
	(next(TraceExp, Event, NewTraceExp)
	 -> (writeln('new:'),writeln(NewTraceExp), log('matched event #'), log(EventId), lognl, NewEventId is EventId+1, verify(TraceStream, NewTraceExp, NewEventId))
	 ;  (log('ERROR on event '), dict_pairs(Event, _, Fields), log(Fields), lognl, false)
	))).

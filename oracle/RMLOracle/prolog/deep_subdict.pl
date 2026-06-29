:- module('deep_subdict', [deep_subdict/2]).

% three cases: dictionaries, lists and others

% fix by Davide (Jan 22, 2020)
% deep_subdict(V,_{x:1,y:2}). should succeeds with V = _7026{x:1, y:2} and not V = _7276{}

% Davide (Jun 8, 2020)
% changed arguments order, in accordance with the match predicate

deep_subdict(Event, Pattern) :- var(Pattern),!,Pattern=Event.

% if the pattern is empty we're done
deep_subdict(_{}, Event) :- is_dict(Event),!.

% dictionaries
deep_subdict(Pattern, Event) :-
	% ensure they're dicts
	is_dict(Pattern),
	is_dict(Event),
	!,
	% take a pair out from the pattern
	dict_pairs(Pattern, _, [Key-PatternValue|OtherPatternPairs]),
	% find a pair with the same key in the event
	dict_pairs(Event, _, EventPairs),
	member(Key-EventValue, EventPairs),
	% check wether values are compatible
	deep_subdict(EventValue,PatternValue),
	% drop the already checked pair from the subdict
	dict_pairs(PatternRest, _, OtherPatternPairs),
	% check the rest of the dict
	deep_subdict(PatternRest, Event).

% lists of events and patterns
deep_subdict([], []) :- !.
deep_subdict([Event|OtherEvents],[Pattern|OtherPatterns]) :-
	!,
	deep_subdict(Event, Pattern),
	deep_subdict(OtherEvents, OtherPatterns).

% others
deep_subdict(X, X).

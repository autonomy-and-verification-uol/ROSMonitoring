:- module('deep_subdict', [deep_subdict/2]).

% three cases: dictionaries, lists and others

% if the first dict is empty we're done
deep_subdict(_{}, D) :- is_dict(D).

% dictionaries
deep_subdict(D1, D2) :-
	% ensure they're dicts
	is_dict(D1),
	is_dict(D2),
	!,
	% take a pair out from the subdict
	dict_pairs(D1, _, [K-V1|L1]),
	% find a pair with the same key in the superdict
	dict_pairs(D2, _, L2),
	member(K-V2, L2),
	% check wether values are compatible
	deep_subdict(V1, V2),
	% drop the already checked pair from the subdict
	dict_pairs(D1rest, _, L1),
	% check the rest of the dict
	deep_subdict(D1rest, D2).

% lists
deep_subdict([], []) :- !.
deep_subdict([X1|L1], [X2|L2]) :-
	!,
	deep_subdict(X1, X2),
	deep_subdict(L1, L2).

% others
deep_subdict(X, X).
deep_subdict(X, Y) :-
	string(X), not(string(Y)),
	term_string(Y, YStr),
	X = YStr.
deep_subdict(X, Y) :-
	string(Y), not(string(X)),
	term_string(X, XStr),
	XStr = Y.

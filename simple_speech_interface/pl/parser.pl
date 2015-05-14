%%%%%%%%%%
% How to run it: call swipl and load the file by calling "[parser]."
% Test query: "command(Action,Object,[grab,the,red,box],[])."
%
parse(A, Action, Object) :- command(Action, Object, A, []).
parse(A, Action, _) :- command(Action, A, []).

command(Action) --> verb(Action).

command(Action, Object) --> verb(Action), reference(Object).

verb(Action) --> [Action], {action(Action)}.
verb(Action) --> [A], [B], {atom_concat(A, B, C), action(C), Action = C}. 
verb(Action) --> [A], {not(action(A)), atom_concat(unknown_action_, A, C), Action = C}.


%%%%%
% property unknown
%%%%%
reference(ObjectID) --> [Art], [Property], [_], {det(Art), not(property(Property)), atom_concat(unknown_property_, Property, C), ObjectID = C}.

%%%%%
% everything is grounded
%%%%%
reference(ObjectID) --> [Art], [Property], [Object], {det(Art), property(Property), object(Object), property(ObjectID, Property), objectType(ObjectID, Object)}.

%%%%%
% object unseen
%%%%%
reference(ObjectID) --> [Art], [Property], [Object], {det(Art), property(Property), object(Object), not(property(ObjectID, Property)), objectType(_, Object), atom_concat(object_unseen_, Object, C), ObjectID = C}.

reference(ObjectID) --> [Art], [Property], [Object], {det(Art), property(Property), not(object(Object)), atom_concat(unknown_object_, Object, C), ObjectID = C}.


reference(Object) --> [Art], [ObjName], {det(Art), objectType(Object, ObjName)}.
reference(Object) --> [ObjName], {objectType(Object, ObjName)}.
reference(Object) --> [_], {Object = unknown_object}.

action(grab).
action(pickup).
action(lift).
action(point).
action(touch).
action(start).

det(the).
det(a).
det(an).

property(blue).
property(red).
property(first).

object(box).

objectType(box1, box).
property(box1, red).

property(_, unknown_property).

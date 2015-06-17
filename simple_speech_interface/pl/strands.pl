%%%%%%%%%%
% How to run it: call swipl and load the file by calling "[parser]."
% Test query: "command(Action,Object,[grab,the,red,box],[])."
%

parse(A, FrameRep) :- command(FrameRep, A, []).

command(FrameRep) --> verb(Action), object(Obj), destination(Dest), {atom_concat(Action, '(', C), atom_concat(C, Obj, D), atom_concat(D, ',', E), atom_concat(E, Dest, F), atom_concat(F, ')', FrameRep) }.

verb(Action) --> [Action], {action(Action)}.
object(Obj)  --> [Obj], {object(Obj)}.
destination(Dest)  --> [Dest], {destination(Dest)}.


action(deliver).
object(book).
destination(nick).
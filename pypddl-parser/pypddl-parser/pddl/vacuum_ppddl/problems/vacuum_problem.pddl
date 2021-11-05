(define (problem vacuum-all)

(:domain simple-vacuum-mdp)

(:requirements :negative-preconditions)

(:init ( and (left-dirty) (right-dirty) (at-left) ))

(:goal ( and (not (left-dirty) (not (right-dirty)) ) )

)


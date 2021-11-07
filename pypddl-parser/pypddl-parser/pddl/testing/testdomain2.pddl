(define (domain testdomain2)

	(:requirements :strips :equality :typing)

	(:types cell)

	(:predicates
		(robot-at ?x - cell)
		(dirty-at ?x - cell)
	)

	(:action move
		:parameters (?x ?y - cell)
		:precondition (and (robot-at ?x) (not (= ?x ?y)))
		:effect (and (robot-at ?y) (not (robot-at ?x)))
	)

	(:action clean
		:parameters (?x - cell)
		:precondition (and (robot-at ?x) (dirty-at ?x))
		:effect (not (dirty-at ?x))
	)

)
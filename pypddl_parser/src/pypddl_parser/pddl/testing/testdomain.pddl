;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Testing to find parser gaps
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; PDDL Domain Specification File

(define (domain testdomain)

	(:requirements :strips :equality :typing)

	(:types cell)

	(:predicates
		(robot-at ?x - cell)
	)

	(:action move
		:parameters (?x ?y - cell)
		:precondition (and (robot-at ?x) (not (= ?x ?y)))
		:effect (and (robot-at ?y) (not (robot-at ?x)))
	)

)
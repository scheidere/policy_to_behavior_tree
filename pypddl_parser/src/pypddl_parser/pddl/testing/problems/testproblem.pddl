;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Testing to find parser gaps
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(define (problem testproblem)
	
	(:domain testdomain)
	
	(:objects
		left-cell right-cell - cell
	)

	(:init
		(robot-at left-cell)
	)

	(:goal
		(and
			(robot-at right-cell)
		)
	)
)

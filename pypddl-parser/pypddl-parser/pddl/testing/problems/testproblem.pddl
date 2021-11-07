;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Testing to find parser gaps
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(define (problem box01)
	
	(:domain robot)
	
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

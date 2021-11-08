(define (problem testproblem2)
	
	(:domain testdomain2)
	
	(:objects
		left-cell right-cell - cell
	)

	(:init
		(robot-at left-cell)
		(dirty-at left-cell)
		(dirty-at right-cell)
	)

	(:goal
		(and
		    (robot-at right-cell)
			;(not (dirty-at left-cell))
			;(not (dirty-at right-cell))
		)
	)
)

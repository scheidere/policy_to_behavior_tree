(define (problem testproblem2)
	
	(:domain testdomain2)
	
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

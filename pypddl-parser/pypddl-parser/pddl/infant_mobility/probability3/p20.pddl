(define (domain infant-p30)

    (:requirements :equality :typing :probabilistic-effects)

    (:types None activity distance)

    (:predicates
        (has-bubbles ?x - None)
        (infant-activity ?x - activity)
        (infant-robot-distance ?x - distance)
    )

    (:constants
        direct-social-interaction social-interaction solitary-play - distance
        moving-toward moving-away stationary - activity
    )
    
    (:constraints
        (and
            (forall (?act - activity) (at-most-once (infant-activity ?act)))
            (forall (?dst - distance) (at-most-once (infant-robot-distance ?dst)))
        )
    )

    (:action spin

        :parameters ()
        :precondition (or (infant-robot-distance direct-social-interaction) (infant-robot-distance social-interaction) (infant-robot-distance solitary-play)) ;(or (infant-robot-distance direct-social-interaction) (infant-robot-distance social-interaction) (infant-robot-distance solitary-play)) ;(or (direct-social-interaction) (social-interaction) (solitary-play))
        :effect (probabilistic 0.8 (and (infant-activity moving-toward) (increase reward 2))
                               0.2 (increase reward -2)
                )
    )

    (:action move-toward

        :parameters ()
        :precondition (not (infant-robot-distance direct-social-interaction))
        :effect (probabilistic 0.8 (and (infant-activity moving-toward) (increase reward 2))
                               0.2 (increase reward -2)
                )
    )

    (:action move-away

        :parameters ()
        :precondition (or (infant-robot-distance direct-social-interaction) (infant-robot-distance social-interaction) (infant-robot-distance solitary-play)) ;(or (direct-social-interaction) (social-interaction) (solitary-play))
        :effect (probabilistic 0.8 (and (infant-activity moving-toward) (increase reward 2))
                               0.2 (increase reward -2)
                )
    )

    (:action blow-bubbles

        :parameters (?x - None)
        :precondition (and (infant-robot-distance social-interaction) (has-bubbles ?x))
        :effect (probabilistic 0.8 (and (infant-activity moving-toward) (increase reward 2))
                               0.15 (increase reward -2)
                               0.05 (and (not (has-bubbles ?x)) (increase reward -2))
                )
    )

    (:action refill-bubbles

        :parameters (?x - None)
        :precondition (not (has-bubbles ?x))
        :effect (and (has-bubbles ?x))
    )

    (:action play-sounds

        :parameters ()
        :precondition (not (infant-activity moving-toward))
        :effect (probabilistic 0.8 (and (infant-activity moving-toward) ( increase reward 2))
                               0.2 (increase reward -2)
                )
    )

    (:action lights-and-sounds

        :parameters ()
        :precondition (or (infant-robot-distance direct-social-interaction) (infant-robot-distance social-interaction) (infant-robot-distance solitary-play))
        :effect (probabilistic 0.8 (and (infant-activity moving-toward) ( increase reward 2))
                               0.2 (increase reward -2)
                )
    )

    (:action keep-away

        :parameters ()
        :precondition (infant-robot-distance direct-social-interaction)
        :effect (probabilistic 0.8 (and (infant-robot-distance social-interaction) (infant-activity moving-toward) (increase reward 2))
                               0.1 (and (infant-robot-distance social-interaction) (infant-activity stationary) (increase reward -2)) 
                               0.1 (and (infant-robot-distance solitary-play) (infant-activity stationary) (increase reward -2))
                )
    )

    (:action go-to-infant

        :parameters ()
        :precondition (not (infant-robot-distance direct-social-interaction)) ; solitary play or social interaction distances
        :effect (probabilistic 0.8 (and (infant-robot-distance direct-social-interaction) (infant-activity moving-toward) (increase reward 2))
                               0.1 (and (infant-robot-distance direct-social-interaction) (infant-activity moving-away) (increase reward -2))
                               0.1 (and (infant-robot-distance social-interaction) (infant-activity moving-away) (increase reward -2))
                )
    )

)
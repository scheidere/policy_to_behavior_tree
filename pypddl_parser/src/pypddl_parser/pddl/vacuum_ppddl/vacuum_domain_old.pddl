( define ( domain simple-vacuum-mdp )

  ( :requirements :negative-preconditions  :conditional-effects :rewards ) ; :rewards

  ( :predicates ( at-left ) ( left_dirty ) ( right_dirty ) )

  ( :action clean
	:effect (
		( probabilistic 0.9 ( and 
					( when
						; Antecedent
						( and ( at-left ) ( left-dirty )  )
						; Consequence
						(and ( increase reward 2 )
						        ( not (left_dirty) )  ; check this
						)
					)
					( when
						; Antecedent
						( and ( not ( at-left ) ) ( right-dirty )  )
						; Consequence
						(and ( increase reward 2 )
						       ( not ( right_dirty ) )  ; check this
						)
					) 
        				  ) ; end and

					  0.1 ( ( increase reward -1 ) ) ; Actual format ( increase (reward) -1 )
      	) ; end probabilistic
	) ; end effect

) ; action end

( :action move-left
	:precondition ( not ( at-left ) ) 
	:effect ( ( at-left ) )
) 

( :action move-right
	:precondition ( ( at-left ) ) 
	:effect ( (not ( at-left )) )
) 

) ; domain end


;; problem file: cleaner-problem.pddl

(define (problem cleaner-problem)

    (:domain cleaner-domain)

    (:objects 
        agent0 - robot
		bathroom - waypoint
        bedroom - waypoint
        kitchen - waypoint
        dock - waypoint
    )

	(:init 
        (workfree agent0)
        (in agent0 dock)
        (recharging_station dock)
		(= (battery_charge) 90)
    )

    (:goal 
        (and 
            (cleaned bathroom)
            (cleaned bedroom)
            (cleaned kitchen)
        )
    )

)
;; domain file: turtle-domain.pddl

(define (domain turtle-domain)

    (:requirements :strips :typing :adl :fluents :durative-actions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        turtle
        controller
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    (:predicates 
        (control ?c - controller ?t - turtle)
        (target ?c - controller ?t - turtle)
    )

    (:durative-action movetotarget
        :parameters (?disk ?from ?to - disk ?a - arm)
        :duration (= ?duration 5)
        :condition (and
            (at start(clear ?disk))
            (at start(clear ?to))
            (at start(on ?disk ?from))
            (at start(smaller ?disk ?to))
            (at start(not_moving ?a))
        )
        :effect (and
            (at start(clear ?from))
            (at start(not(on ?disk ?from)))
            (at start(not(not_moving ?a)))

            (at end(not_moving ?a))
            (at end(on ?disk ?to))
            (at end(not(clear ?to)))

        )
    )

)

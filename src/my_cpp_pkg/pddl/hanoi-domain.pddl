;; domain file: hanoi-domain.pddl

(define (domain hanoi-domain)

    (:requirements :strips :typing :adl :fluents :durative-actions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        disk
        arm
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    (:predicates 
        (clear ?disk - disk)
        (on ?disk1 ?disk2 - disk)
        (smaller ?disk1 ?disk2 - disk)
        (not_moving ?a - arm)
    )

    (:durative-action moveontop
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

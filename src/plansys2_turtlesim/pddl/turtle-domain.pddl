;; domain file: turtle-eat-domain.pddl

(define (domain turtle-eat-domain)

    (:requirements :strips :typing :adl :fluents :durative-actions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        pos
        turtle
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    (:predicates 
        (control ?t - turtle)
        (targetfree ?t - turtle)
        (currenttarget ?t - turtle)
        (alive ?t - turtle)
        (eaten ?t - turtle)
        (posx ?t - turtle ?p - pos)
        (posy ?t - turtle ?p - pos)
    )

    (:functions
        (turtleseaten ?t - turtle)
        (distance ?t1 ?t2 - turtle)
    )

    (:durative-action acquiretarget
        :parameters (?t1 ?t2 - turtle)
        :duration (= ?duration 1)
        :condition (and
            (at start (targetfree ?t1))
            (over all (control ?t1))
            (over all (alive ?t2))
        )
        :effect (and
            (at start (currenttarget ?t2))
            (at start (not(targetfree ?t1)))
        )
    )

    (:durative-action movetoward
        :parameters (?t1 ?t2 - turtle ?p2x ?p2y - pos)
        :duration (= ?duration 4)
        :condition (and
            (at start (> (distance ?t1 ?t2) 0))
            (over all (control ?t1))
            (over all (posx ?t2 ?p2x))
            (over all (posy ?t2 ?p2y))
            (over all (currenttarget ?t2))
            (over all (alive ?t2))
        )
        :effect (and
            (at end (assign (distance ?t1 ?t2) 0))
            (at end (posx ?t1 ?p2x))
            (at end (posy ?t1 ?p2y))
        )
    )

    (:durative-action eating
        :parameters (?t1 ?t2 - turtle)
        :duration (= ?duration 2)
        :condition (and
            (over all (control ?t1))
            (over all (currenttarget ?t2))
            (over all (alive ?t2))
            (over all (< (distance ?t1 ?t2) 1))
        )
        :effect (and
            (at end (not(alive ?t2)))
            (at end (eaten ?t2))
            (at end (not (currenttarget ?t2)))
            (at end (targetfree ?t1))
            (at end (increase (turtleseaten ?t1) 1))
        )
    )

)

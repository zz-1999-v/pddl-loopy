(define (domain pick-and-place)
  (:requirements :strips :equality)
  (:predicates
    (Stackable ?o ?r)
    (Sink ?r)
    (Stove ?r)

    (Pose ?o ?p)
    (Grasp ?o ?g)
    (Kin ?o ?p ?g ?q ?t)
    (FreeMotion ?q1 ?t ?q2)
    (HoldingMotion ?q1 ?t ?q2 ?o ?g)
    (Supported ?o ?p ?r)
    (Traj ?t)

    (TrajCollision ?t ?o2 ?p2)
    (CFreePosePose ?o ?p ?o2 ?p2)
    (CFreeApproachPose ?o ?p ?g ?o2 ?p2)
    (CFreeTrajPose ?t ?o2 ?p2)

    (AtPose ?o ?p)
    (AtGrasp ?o ?g)
    (HandEmpty)
    (AtConf ?q)
    (CanMove)
    (Pick_gaizi ?o ?r)
    (Restart_pick ?o ?r)
    (In_region ?o ?r)
    (Block1_in_region ?o ?r)
    (Block2_in_region ?o ?r)
    (Cooked ?o ?r)
    (Occupied ?r)
    (Refuse_pick)
    (Drill ?o ?r)
    (Cleared_slot ?o ?o2 ?r2)
    (Pick_gaizi_to_target_region ?o3 ?r)

    (Block1 ?o)
    (Block2 ?o)
    (Gaizi ?o)

    (On ?o ?r)
    (Holding ?o)

    (UnsafePose ?o ?p)
    (UnsafeApproach ?o ?p ?g)
    (UnsafeTraj ?t)
  )

  (:action move_free
    :parameters (?q1 ?q2 ?t)
    :precondition (and (FreeMotion ?q1 ?t ?q2)
                       (AtConf ?q1) (HandEmpty) (CanMove)
                       ;(not (UnsafeTraj ?t))
                  )
    :effect (and (AtConf ?q2)
                 (not (AtConf ?q1)) (not (CanMove)))
  )
  (:action move_holding
    :parameters (?q1 ?q2 ?o ?g ?t)
    :precondition (and (HoldingMotion ?q1 ?t ?q2 ?o ?g)
                       (AtConf ?q1) (AtGrasp ?o ?g) (CanMove)
                       ;(not (UnsafeTraj ?t))
                  )
    :effect (and (AtConf ?q2)
                 (not (AtConf ?q1)) (not (CanMove)))
  )

  (:action pick
    :parameters (?o ?p ?g ?q ?t)
    :precondition (and (Kin ?o ?p ?g ?q ?t)
                       (AtPose ?o ?p) (HandEmpty) (AtConf ?q)
                       (not (UnsafeApproach ?o ?p ?g))
                       (not (UnsafeTraj ?t))
                  )
    :effect (and (AtGrasp ?o ?g) (CanMove)
                 (not (AtPose ?o ?p)) (not (HandEmpty)))
  )
  (:action place
    :parameters (?o ?p ?g ?q ?t)
    :precondition (and (Kin ?o ?p ?g ?q ?t)
                       (AtGrasp ?o ?g) (AtConf ?q)
                       (not (UnsafePose ?o ?p))
                       (not (UnsafeApproach ?o ?p ?g))
                       (not (UnsafeTraj ?t))
                  )
    :effect (and (AtPose ?o ?p) (HandEmpty) (CanMove)
                 (not (AtGrasp ?o ?g)))
  )
  (:action move_block1_to_region
    :parameters (?o ?r ?r2 ?p)
    :precondition (and (Block1_in_region ?o ?r) (Stove ?r2)
                        (Block1 ?o)
                        (Occupied ?r)
                        (On ?o ?r2))
    :effect  (not (Block1_in_region ?o ?r))            
  )

  (:action move_block2_to_region
    :parameters (?o ?o2 ?r ?r2)
    :precondition (and (Block2_in_region ?o ?r) (Stove ?r2) (On ?o ?r2)
                        (Block2 ?o)
                        (not (Block1_in_region ?o2 ?r)) (Block1 ?o2)
                        (Occupied ?r))
    :effect (not (Block2_in_region ?o ?r))
  )

  (:action clear_slot
    :parameters (?o ?r)
    :precondition (and (not (Block2_in_region ?o ?r)) (Block2 ?o)
                      (Occupied ?r))
    :effect (not (Occupied ?r))
  )


  (:action start_to_pick_gaizi
    :parameters (?o ?r)
    :precondition (and (not (Occupied ?r))
                      (Gaizi ?o))                
    :effect (Pick_gaizi ?o ?r)        
  )

  (:action drill
    :parameters (?o ?r)
    :precondition (and (On ?o ?r) (On ?o ?r) (On ?o ?r) (On ?o ?r))
    :effect (Drill ?o ?r)
  )

  (:derived (On ?o ?r)
    (exists (?p) (and (Supported ?o ?p ?r)
                      (AtPose ?o ?p)))
  )
  (:derived (Holding ?o)
    (exists (?g) (and (Grasp ?o ?g)
                      (AtGrasp ?o ?g)))
  )

  (:derived (UnsafePose ?o ?p)
    (exists (?o2 ?p2) (and (Pose ?o ?p) (Pose ?o2 ?p2) (not (= ?o ?o2))
                           (not (CFreePosePose ?o ?p ?o2 ?p2))
                           (AtPose ?o2 ?p2)))
  )
  (:derived (UnsafeApproach ?o ?p ?g)
    (exists (?o2 ?p2) (and (Pose ?o ?p) (Grasp ?o ?g) (Pose ?o2 ?p2) (not (= ?o ?o2))
                           (not (CFreeApproachPose ?o ?p ?g ?o2 ?p2))
                           (AtPose ?o2 ?p2)))
  )
  (:derived (UnsafeTraj ?t)
    (exists (?o2 ?p2) (and (Traj ?t) (Pose ?o2 ?p2)
                           (not (CFreeTrajPose ?t ?o2 ?p2))
                           ; (TrajCollision ?t ?o2 ?p2)
                           (AtPose ?o2 ?p2)))
  )
)
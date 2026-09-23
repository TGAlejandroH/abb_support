MODULE TG_Parts
    !***********************************************************************
    ! Production Manager part registrations for the TetraGen vision system.
    !
    ! Design: TGuideWeldingHMI/docs/socket_start_trigger_hmi_plan_v1.md
    !         S1  - arming is "TG as a Production Manager part"
    !         S-C - one partdata + partadv + wrapper per workpiece family
    !               PER STATION (see "registration pattern" below)
    !
    ! THIS IS A RESIDENT MODULE. It must be on the controller and loaded
    ! BEFORE the operator selects a part - it is NOT the generated .mod that
    ! the HMI transfers per run. The ordering is absolute: the part has to be
    ! selectable before START is pressed, but the generated module does not
    ! arrive until after the HMI connects, which is after START.
    !
    ! ----------------------------------------------------------------------
    ! What Production Manager does around our routine - VERIFIED 2026-09-22
    ! by reading the positioner package's event hooks (Irbp1EEv.sys) and
    ! procedures (Irbp1Prc.sys) in the function package this controller
    ! loads, and cross-checked against the customer's RW 6.16 programs:
    !
    !   EE_PRE_PROD  PreFetchPadv1  copy next parts' partadv -> padvStn1/2
    !   EE_CLOSE_JIG ArmsToZero_DPos  (customer Utility.sys, real cell only)
    !                               levels BOTH tilt arms to 0 before index
    !   EE_INDEX     EvIndexToStn1  ActInterch1; IndexToStn1; DeactInterch1
    !                               -> rotates the turntable AND, in the same
    !                                  MoveAbsJ, pre-positions both chucks
    !   EE_PRE_PART  EvActStn1      ActStn1 -> ActUnit + MechUnitLoad
    !   (the part routine - our wrapper below)
    !   EE_POST_PART EvDeactStn1    DeactStn1
    !
    ! So PM owns indexing, chuck pre-positioning, activation and load. The
    ! .tgs program owns everything after activation, tilt included.
    !
    ! ----------------------------------------------------------------------
    ! partadv fields, by their REAL names and in their REAL ORDER:
    !     [ procAngle, loadAngle, serviceAngle, load ]
    ! ORDER MEASURED ON THE VC 2026-09-22, not assumed - an earlier revision of
    ! this header had procAngle/loadAngle swapped, which made PM look as if it
    ! ignored partadv. Two independent observations settle it:
    !   IndexToStn1 put station 2's chuck at 60 = 9022's SECOND array, which
    !   IndexToStn1 can only reach through loadAngle; and IndexToStn2 then put
    !   station 1's chuck at 30 (9011's second array, loadAngle) and station
    !   2's at 0 (9022's first array, procAngle), rotating the turntable 180.
    ! CONFIRMED 2026-09-23 from the controller's type definition over RWS
    ! (/rw/rapid/symbol/properties/RAPID/partadv/<component> -> comnum):
    ! procAngle=1, loadAngle=2, serviceAngle=3 (extjoint), load=4 (loaddata).
    ! Only the CHUCK component (eax_c) of procAngle/loadAngle is consumed on
    ! the dispatch path, by IndexToStn1/2:
    !     robot-side station's chuck    := its procAngle.eax_c  (FIRST array)
    !     operator-side station's chuck := its loadAngle.eax_c  (SECOND array)
    ! INTERCH's own jointtarget reads rax_1 = index, rax_2 = station 1 chuck,
    ! rax_3 = station 2 chuck.
    ! Tilt (eax_b) is NOT read on the dispatch path - the customer keeps it 0
    ! and levels the arms at EE_CLOSE_JIG. The positioner's service menus
    ! "Move station n to load/process/service position" DO move tilt AND chuck
    ! of the named angle, so a non-zero tilt here is honoured there. load is applied by
    ! ActStn1 (MechUnitLoad) and matters for positioner motion safety.
    !
    ! ----------------------------------------------------------------------
    ! Registration pattern - copied from the customer's mDeclarations.sys:
    ! every workpiece is registered ONCE PER STATION, odd part number for
    ! station 1 and even for station 2, each with a station-specific routine:
    !     pd_r462509Lst1 := ["RS462509L_Stn1","","",1,1001,"","padv0Stn1"];
    !     pd_r462509Lst2 := ["RS462509L_Stn2","","",2,1002,"","padv0Stn2"];
    ! partdata field 4 is the STATION. A TG part registered for station 1
    ! can only be dispatched for station 1.
    !
    ! ----------------------------------------------------------------------
    ! Why the wrappers. PM late-binds the routine named in partdata field 1.
    ! Giving each part+station its own wrapper makes the dispatched routine BE
    ! the part identity AND the station - no giJobSel read, no
    ! GAP_CURRENT_PART semantics. Both are sent to the HMI in the START line.
    !
    ! !!! DEPLOY HAZARD - READ BEFORE RELOADING THIS MODULE (VC 2026-09-23) !!!
    ! PM reads each partadv LIVE at every dispatch, and the pendant menus
    ! "Set/Change load/process/service position" write the taught values
    ! straight into the partadv PERS declared HERE - in controller memory only.
    ! Reloading this file (repo or HOME:/TGS) therefore erases what an operator
    ! taught, and the old angles apply on the NEXT cycle, silently. Save the
    ! controller's copy out first (RWS modules/TG_Parts?action=save) and diff.
    ! Planned fix: move partdata/partadv into a data-only module deploys never
    ! reload (plan Q-26) - the customer does the same with mDeclarations.sys.
    !
    ! FORMATTING: every aggregate on ONE line. RAPID rejects "!" comments
    ! inside an unterminated expression (40322 Load error, VC 2026-09-22).
    !***********************************************************************

    ! ======================================================================
    ! VC TEST PART 9011 - STATION 1
    ! procAngle chuck 0 deg (first array), loadAngle chuck 30 deg (second),
    ! load 80 kg. procAngle 0 matches the TG program under test, which holds
    ! the chuck at 0 - so PM presents the station at the angle the program
    ! expects. The 30 shows up when station 1 is sent to the operator side.
    ! ======================================================================
    TASK PERS partdata pdTG_9011:=["TG_Part9011","TG vision - VC test part, station 1","",1,9011,"","padvTG_9011"];
    PERS partadv padvTG_9011:=[[0,0,0,0,0,0],[0,0,30,0,0,0],[0,0,0,0,0,0],[80,[0,0,0.15],[1,0,0,0],5,5,8]];

    ! ======================================================================
    ! VC TEST PART 9022 - STATION 2
    ! procAngle chuck 0 deg (first array), loadAngle chuck 60 deg (second),
    ! load 220 kg. NOTE: selecting this part does NOT make PM run station 2 -
    ! PM chooses the station itself (its next-station signals are locked by
    ! the controller's safety access restriction, C0048407), and on this VC it
    ! always chose station 1, indexing back even after station 2 had been
    ! presented from the Service menu.
    ! ======================================================================
    TASK PERS partdata pdTG_9022:=["TG_Part9022","TG vision - VC test part, station 2","",2,9022,"","padvTG_9022"];
    PERS partadv padvTG_9022:=[[0,0,0,0,0,0],[0,0,60,0,0,0],[0,0,0,0,0,0],[220,[0,0,0.3],[1,0,0,0],20,20,35]];

    ! ======================================================================
    ! The wrappers. Stamp identity + station, then run the common routine.
    ! ======================================================================

    PROC TG_Part9011()
        nTG_PartNo:=9011;
        nTG_PartStn:=1;
        TG_VisionOnce;
    ENDPROC

    PROC TG_Part9022()
        nTG_PartNo:=9022;
        nTG_PartStn:=2;
        TG_VisionOnce;
    ENDPROC

ENDMODULE
